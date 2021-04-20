# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math
import os, time, datetime

import rclpy

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, QuaternionStamped
from transforms3d.euler import euler2quat as quaternion_from_euler
from libnmea_navsat_driver.checksum_utils import check_nmea_checksum
from libnmea_navsat_driver import parser

received_time_is_nan = 1
received_utc_time = 0.0
previous_received_utc_time = 0.0
        
gst_rms = 0.0
lat_error=0.0
lon_error=0.0
rmc_lon = 0.0
rmc_lat = 0.0
hdop_GPGSA = 0.0
pdop_GPGSA = 0.0
vdop_GPGSA = 0.0

hdop_GAGSA = 0.0
pdop_GAGSA = 0.0
vdop_GAGSA = 0.0

hdop_GLGSA = 0.0
pdop_GLGSA = 0.0
vdop_GLGSA = 0.0
hdop_GGA = 0.0
GPGSA_fix = 0
SNR1=[100]*4
SNR2=[100]*4
SNR3=[100]*4
SNR4=[100]*4
out_cov = 0.0

utc_time = 0.0
age_of_diff = 0.0

kph_speed = 0.0

gga_longitude = 0.0
gga_latitude = 0.0
gga_altitude = 0.0

rmc_longitude = 0.0
rmc_latitude = 0.0
rmc_altitude = 0.0

GGA_received = 0
VTG_received = 0
q_old = [0.0]*4
started=1
startTime=time.time()
trustedCount = 0
fixCount = 0
badFlag = 10
gps_line = ''
gps_line_first_half = ''
gps_line_second_half = ''

use_gps_date_once = 1

publish_gps = 1
    
satNo = 0
gps_qual = 0.0
gpsQuality=0 #Variable to hold our gps quality 2:neither fix nor float 3:untrusted fix 4:trusted fix  , 5:trusted float  6:semi-trusted_float 7:untrusted_float  
print( "Starting GPS")

name_1 = datetime.datetime.now().strftime("%D").replace('/', '')
name_2 = str(time.time()).split('.')[0]
csv_name = name_1 + '_' + name_2 + '.csv'
#file_name = "/home/jetson/csv/GPS_LINES_" + str(int(time.time())) + ".csv" #TODO rmc date instead of time
file_name = "/home/jetson/csv/" + csv_name


class Ros2NMEADriver(Node):
    def __init__(self):
        super().__init__('nmea_navsat_driver')

        self.trustedFix_pub = self.create_publisher(NavSatFix, 'trusted_fix', 10)
        self.unTrustedFix_pub = self.create_publisher(NavSatFix, 'unTrusted_fix', 10)

        self.trustedFloat_pub = self.create_publisher(NavSatFix, 'trusted_float', 10)
        self.semiTrustedFloat_pub = self.create_publisher(NavSatFix, 'semiTrusted_float', 10)
        self.unTrustedFloat_pub = self.create_publisher(NavSatFix, 'unTrusted_float', 10)

        self.allMsg_pub = self.create_publisher(Odometry, 'allMsg', 10)
                
        self.vel_pub = self.create_publisher(TwistStamped, 'vel', 10)
        self.heading_pub = self.create_publisher(QuaternionStamped, 'heading', 10)
        self.time_ref_pub = self.create_publisher(TimeReference, 'time_reference', 10)

        self.time_ref_source = self.declare_parameter('time_ref_source', 'gps').value
        self.use_RMC = self.declare_parameter('useRMC', False).value
        self.valid_fix = False

        # epe = estimated position error
        self.default_epe_quality0 = self.declare_parameter('epe_quality0', 1000000).value
        self.default_epe_quality1 = self.declare_parameter('epe_quality1', 4.0).value
        self.default_epe_quality2 = self.declare_parameter('epe_quality2', 0.1).value
        self.default_epe_quality4 = self.declare_parameter('epe_quality4', 0.02).value
        self.default_epe_quality5 = self.declare_parameter('epe_quality5', 4.0).value
        self.default_epe_quality9 = self.declare_parameter('epe_quality9', 3.0).value

        self.using_receiver_epe = False

        self.lon_std_dev = float("nan")
        self.lat_std_dev = float("nan")
        self.alt_std_dev = float("nan")

        """Format for this dictionary is the fix type from a GGA message as the key, with
        each entry containing a tuple consisting of a default estimated
        position error, a NavSatStatus value, and a NavSatFix covariance value."""
        self.gps_qualities = {
            # Unknown
            -1: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # Invalid
            0: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # SPS
            1: [
                self.default_epe_quality1,
                NavSatStatus.STATUS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # DGPS
            2: [
                self.default_epe_quality2,
                NavSatStatus.STATUS_SBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Fix
            4: [
                self.default_epe_quality4,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Float
            5: [
                self.default_epe_quality5,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # WAAS
            9: [
                self.default_epe_quality9,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ]
        }

    # Returns True if we successfully did something with the passed in
    # nmea_string
    def add_sentence_original(self, nmea_string, frame_id, timestamp=None):

        if not check_nmea_checksum(nmea_string):
            self.get_logger().warn("Received a sentence with an invalid checksum. " +
                                   "Sentence was: %s" % nmea_string)
            return False

        parsed_sentence = parser.parse_nmea_sentence(nmea_string)
        if not parsed_sentence:
            self.get_logger().debug("Failed to parse NMEA sentence. Sentence was: %s" % nmea_string)
            return False

        if timestamp:
            current_time = timestamp
        else:
            current_time = self.get_clock().now().to_msg()

        current_fix = NavSatFix()
        current_fix.header.stamp = current_time
        current_fix.header.frame_id = frame_id
        current_time_ref = TimeReference()
        current_time_ref.header.stamp = current_time
        current_time_ref.header.frame_id = frame_id
        if self.time_ref_source:
            current_time_ref.source = self.time_ref_source
        else:
            current_time_ref.source = frame_id

        if not self.use_RMC and 'GGA' in parsed_sentence:
            current_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            data = parsed_sentence['GGA']
            fix_type = data['fix_type']
            if not (fix_type in self.gps_qualities):
                fix_type = -1
            gps_qual = self.gps_qualities[fix_type]
            default_epe = gps_qual[0]
            current_fix.status.status = gps_qual[1]
            current_fix.position_covariance_type = gps_qual[2]
            if current_fix.status.status > 0:
                self.valid_fix = True
            else:
                self.valid_fix = False

            current_fix.status.service = NavSatStatus.SERVICE_GPS
            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            current_fix.latitude = latitude

            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude
            current_fix.longitude = longitude

            # Altitude is above ellipsoid, so adjust for mean-sea-level
            altitude = data['altitude'] + data['mean_sea_level']
            current_fix.altitude = altitude

            # use default epe std_dev unless we've received a GST sentence with epes
            if not self.using_receiver_epe or math.isnan(self.lon_std_dev):
                self.lon_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.lat_std_dev):
                self.lat_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.alt_std_dev):
                self.alt_std_dev = default_epe * 2

            hdop = data['hdop']
            current_fix.position_covariance[0] = (hdop * self.lon_std_dev) ** 2
            current_fix.position_covariance[4] = (hdop * self.lat_std_dev) ** 2
            current_fix.position_covariance[8] = (2 * hdop * self.alt_std_dev) ** 2  # FIXME
            print("add sentence gga")
            self.trustedFix_pub.publish(current_fix)

            if not math.isnan(data['utc_time']):
                current_time_ref.time_ref = rclpy.time.Time(seconds=data['utc_time']).to_msg()
                self.last_valid_fix_time = current_time_ref
                self.time_ref_pub.publish(current_time_ref)

        elif not self.use_RMC and 'VTG' in parsed_sentence:
            data = parsed_sentence['VTG']

            # Only report VTG data when you've received a valid GGA fix as well.
            if self.valid_fix:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)

        elif 'RMC' in parsed_sentence:
            data = parsed_sentence['RMC']

            # Only publish a fix from RMC if the use_RMC flag is set.
            if self.use_RMC:
                if data['fix_valid']:
                    current_fix.status.status = NavSatStatus.STATUS_FIX
                else:
                    current_fix.status.status = NavSatStatus.STATUS_NO_FIX

                current_fix.status.service = NavSatStatus.SERVICE_GPS

                latitude = data['latitude']
                if data['latitude_direction'] == 'S':
                    latitude = -latitude
                current_fix.latitude = latitude

                longitude = data['longitude']
                if data['longitude_direction'] == 'W':
                    longitude = -longitude
                current_fix.longitude = longitude

                current_fix.altitude = float('NaN')
                current_fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.trustedFix_pub.publish(current_fix)

                if not math.isnan(data['utc_time']):
                    current_time_ref.time_ref = rclpy.time.Time(seconds=data['utc_time']).to_msg()
                    self.time_ref_pub.publish(current_time_ref)

            # Publish velocity from RMC regardless, since GGA doesn't provide it.
            if data['fix_valid']:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)
        elif 'GST' in parsed_sentence:
            data = parsed_sentence['GST']

            # Use receiver-provided error estimate if available
            self.using_receiver_epe = True
            self.lon_std_dev = data['lon_std_dev']
            self.lat_std_dev = data['lat_std_dev']
            self.alt_std_dev = data['alt_std_dev']
        elif 'HDT' in parsed_sentence:
            data = parsed_sentence['HDT']
            if data['heading']:
                current_heading = QuaternionStamped()
                current_heading.header.stamp = current_time
                current_heading.header.frame_id = frame_id
                q = quaternion_from_euler(0, 0, math.radians(data['heading']))
                current_heading.quaternion.x = q[0]
                current_heading.quaternion.y = q[1]
                current_heading.quaternion.z = q[2]
                current_heading.quaternion.w = q[3]
                self.heading_pub.publish(current_heading)
        else:
            return False



##### Add the old add_sentence function & modify only publishers #TODO The whole function needs a look

    utc_time = 0.0
    gps_line = ''

    def add_sentence(self, nmea_string, frame_id, timestamp=None):
        global lat_error, lon_error,started, trustedCount, badFlag,fixCount, gst_rms
        global rmc_lon,rmc_lat, hdop_GPGSA, pdop_GPGSA, vdop_GPGSA, GPGSA_fix
        global hdop_GAGSA, pdop_GAGSA, vdop_GAGSA, hdop_GLGSA, pdop_GLGSA, vdop_GLGSA
        global SNR1,SNR2,SNR3, SNR4, hdop_GGA
        global rmc_longitude, rmc_latitude, rmc_altitude, gpsQuality
        global gga_longitude, gga_latitude , gga_altitude
        global kph_speed, age_of_diff
        global out_cov, utc_time, satNo, gps_qual
        global gps_line, gps_line_first_half, gps_line_second_half
        global received_time_is_nan, received_utc_time, previous_received_utc_time
        global VTG_received, GGA_received, use_gps_date_once, file_name, publish_gps

        #if rospy.has_param('publish_gps'):
        #    publish_gps = rospy.get_param('publish_gps')

        trustedFix = NavSatFix()
        unTrustedFix = NavSatFix()
        trustedFloat = NavSatFix()
        semiTrustedFloat = NavSatFix()
        unTrustedFloat = NavSatFix()
        
        allMsg = Odometry()
        trustedFix.header.frame_id = frame_id
        unTrustedFix.header.frame_id = frame_id
        trustedFloat.header.frame_id = frame_id
        semiTrustedFloat.header.frame_id = frame_id
        unTrustedFloat.header.frame_id = frame_id

        allMsg.header.frame_id = frame_id

        if not check_nmea_checksum(nmea_string):
            self.get_logger().warn("Received a sentence with an invalid checksum. " +
                          "Sentence was: %s" % repr(nmea_string))
            return False


        '''
        Publish gps lines to Ros string Msg
        '''
        if "$GN" in nmea_string:
            gps_line_first_half += nmea_string + "," + str(time.time()) + ","
        else:
            gps_line_second_half += nmea_string + "," + str(time.time()) + ","            
                
        if 'GNVTG' in nmea_string:
            gps_line += gps_line_first_half + gps_line_second_half + str(self.get_clock().now().to_msg()) + "\n"
            with open(file_name, 'a') as csvfile:
                csvfile.write(gps_line)

            gps_string_msg = String()
            gps_string_msg.data = gps_line
            self.string_pub.publish(gps_string_msg)
            gps_line = ''
            gps_line_first_half = ''
            gps_line_second_half = ''

        if ('GPGSV' in nmea_string and len(nmea_string) < 100): # This message is received too short when it's empty causing runtime error in parsing (This error appeared in ROS2)
            return
        parsed_sentence = parser.parse_nmea_sentence(nmea_string)
        if not parsed_sentence:
            self.get_logger().debug("Failed to parse NMEA sentence. Sentece was: %s" % nmea_string)
            return False


        if 'GAGSA' in parsed_sentence:
            gsa_data=parsed_sentence['GSA']
            
            if math.isnan(gsa_data['PDop']):
                pdop_GAGSA = 100.0
            else:
                pdop_GAGSA=gsa_data['PDop']

            if math.isnan(gsa_data['HDop']) :
                hdop_GAGSA = 100.0
            else:
                hdop_GAGSA=gsa_data['HDop']

            if math.isnan(gsa_data['VDop']) :
                vdop_GAGSA = 100.0
            else:
                vdop_GAGSA=gsa_data['VDop']

        if 'GLGSA' in parsed_sentence:
            gsa_data=parsed_sentence['GSA']
            
            if math.isnan(gsa_data['PDop']):
                pdop_GLGSA = 100.0
            else:
                pdop_GLGSA=gsa_data['PDop']

            if math.isnan(gsa_data['HDop']) :
                hdop_GLGSA = 100.0
            else:
                hdop_GLGSA=gsa_data['HDop']

            if math.isnan(gsa_data['VDop']) :
                vdop_GLGSA = 100.0
            else:
                vdop_GLGSA=gsa_data['VDop']

        if 'GPGSA' in parsed_sentence:
            gsa_data=parsed_sentence['GSA']
            
            if math.isnan(gsa_data['PDop']):
                pdop_GPGSA = 100.0
            else:
                pdop_GPGSA=gsa_data['PDop']

            if math.isnan(gsa_data['HDop']) :
                hdop_GPGSA = 100.0
            else:
                hdop_GPGSA=gsa_data['HDop']

            if math.isnan(gsa_data['VDop']) :
                vdop_GPGSA = 100.0
            else:
                vdop_GPGSA=gsa_data['VDop']

            if math.isnan(gsa_data['fix']):
                GPGSA_fix = 100
            else:
                GPGSA_fix = gsa_data['fix']

        if 'GPGSV' in parsed_sentence:
            gsv_data=parsed_sentence['GSV']
            msgNo=gsv_data['MSGNo']
            if math.isnan(gsv_data['SNR1']) :
                SNR1[msgNo-1] = 100.0
            else: 
                SNR1[msgNo-1]=gsv_data['SNR1']

            if math.isnan(gsv_data['SNR2']) :
                SNR2[msgNo-1] = 100.0
            else:
                SNR2[msgNo-1]=gsv_data['SNR2']

            if math.isnan(gsv_data['SNR3']) :
                SNR3[msgNo-1] = 100.0
            else:
                SNR3[msgNo-1]=gsv_data['SNR3']

            if math.isnan(gsv_data['SNR4']) :
                SNR4[msgNo-1] = 100.0
            else:
                SNR4[msgNo-1]=gsv_data['SNR4']


        if 'GST' in parsed_sentence:
            gst_data=parsed_sentence['GST']

            if math.isnan(gst_data['ranges_std_dev']):
                gst_rms = 100.0
            else:
                gst_rms = gst_data['ranges_std_dev']

            if math.isnan(gst_data['lat_std_dev']) or math.isnan(gst_data['lon_std_dev']):
                lat_error = 100.0
                lon_error = 100.0
            else:
                lat_error=gst_data['lat_std_dev']
                lon_error=gst_data['lon_std_dev']

        if 'RMC' in parsed_sentence: #20200914 was elif
            data = parsed_sentence['RMC']
            date_line = data['date'] + str("_") + str(int(data['utc_time']))
            #gps_string_date_msg = String()
            #gps_string_date_msg.data = date_line
            #self.string_date_pub.publish(gps_string_date_msg)

            if (use_gps_date_once):
                use_gps_date_once = 0
                file_name = "/home/jetson/csv/GPS_LINES_" + data['date'] + str(int(data['utc_time'])) + ".csv"
                
            rmc_longitude = data['longitude']
            rmc_latitude = data['latitude']
            #rmc_altitude = data['altitude']

            # Publish velocity from RMC regardless, since GGA doesn't provide it.
            if data['fix_valid']:
                current_vel = TwistStamped()
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * \
                    math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * \
                    math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)

        if 'VTG' in parsed_sentence:
            VTG_received = 1
            vtg_data = parsed_sentence['VTG']
            if math.isnan(vtg_data['kph_speed']):
                kph_speed = 100.0
            else:
                kph_speed = vtg_data['kph_speed']


        if 'GGA' in parsed_sentence:
            GGA_received = 1
            data = parsed_sentence['GGA']
            gps_qual = data['fix_type']
            age_of_diff = data['age_of_diff']

            gga_latitude = data['latitude']
            if data['latitude_direction'] == 'S' :
                gga_latitude = -gga_latitude
            gga_longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                gga_longitude = -gga_longitude
            gga_altitude = data['altitude']
            hdop_GGA = data['hdop']
            satNo = data['num_satellites']
            if not math.isnan(data['utc_time']):
                received_utc_time = data['utc_time']
                received_time_is_nan = 0
            else:
                received_time_is_nan = 1



        if VTG_received and GGA_received:
            VTG_received = 0
            GGA_received = 0
            previous_received_utc_time = received_utc_time
            if gps_qual == 0:
                trustedFix.status.status = NavSatStatus.STATUS_NO_FIX
            elif gps_qual == 1:
                trustedFix.status.status = NavSatStatus.STATUS_FIX
            elif gps_qual == 2:
                trustedFix.status.status = NavSatStatus.STATUS_SBAS_FIX
            elif gps_qual in (4, 5):
                trustedFix.status.status = NavSatStatus.STATUS_GBAS_FIX
            elif gps_qual == 9:
                # Support specifically for NOVATEL OEM4 recievers which report WAAS fix as 9
                # http://www.novatel.com/support/known-solutions/which-novatel-position-types-correspond-to-the-gga-quality-indicator/
                trustedFix.status.status = NavSatStatus.STATUS_SBAS_FIX
            else:
                trustedFix.status.status = NavSatStatus.STATUS_NO_FIX

            trustedFix.status.service = NavSatStatus.SERVICE_GPS

            lat_error2 = lat_error*lat_error
            lon_error2 = lon_error*lon_error
            underroot = 0.5*(lat_error2 + lon_error2)

            #print "ALL :::::: 1sigma" , math.sqrt(underroot), ",2sigma" , 2*math.sqrt(underroot), ",3sigma" , 3*math.sqrt(underroot)
            #print "LAT :::::: 1sigma" , math.sqrt(underroot), ",2sigma" , 2*math.sqrt(underroot), ",3sigma" , 3*math.sqrt(underroot)
            #print "LON :::::: 1sigma" , math.sqrt(underroot), ",2sigma" , 2*math.sqrt(underroot), ",3sigma" , 3*math.sqrt(underroot)

            latitude = gga_latitude                                        
            trustedFix.latitude = latitude 
            unTrustedFix.latitude = latitude 
            trustedFloat.latitude = latitude 
            semiTrustedFloat.latitude = latitude 
            unTrustedFloat.latitude = latitude 

            allMsg.pose.pose.position.x = latitude 


            longitude = gga_longitude
            trustedFix.longitude = longitude 
            unTrustedFix.longitude = longitude
            trustedFloat.longitude = longitude 
            semiTrustedFloat.longitude = longitude 
            unTrustedFloat.longitude = longitude

            allMsg.pose.pose.position.y = longitude 


            #31Dec2019
            altitude = gga_altitude
            trustedFix.altitude = altitude 
            unTrustedFix.altitude = altitude
            trustedFloat.altitude = altitude 
            semiTrustedFloat.altitude = altitude 
            unTrustedFloat.altitude = altitude

            allMsg.pose.pose.position.z = altitude 

            newTime = self.get_clock().now().to_msg()

            trustedFix.header.stamp =newTime
            unTrustedFix.header.stamp = newTime
            trustedFloat.header.stamp = newTime
            semiTrustedFloat.header.stamp = newTime
            unTrustedFloat.header.stamp = newTime

            allMsg.header.stamp = newTime

            
            #if (badFlag != 0): 
            #    print (" setting badFlag",badFlag)
            #rospy.set_param('badGpsFlag', badFlag)
            #print (" sdone etting badFlag")
            if gps_qual == 4 and lat_error <= 0.05 and lon_error <= 0.05: #TODO tuning this number 14 Oct 2019
                gpsQuality = 4
                out_cov = 0.00001
                trustedFix.position_covariance[0] = out_cov
                trustedFix.position_covariance[4] = out_cov
                trustedFix.position_covariance[8] = 0
                trustedCount += 2
                if fixCount < 3: # To solve float to fix change in orientation
                    out_cov = 3*math.sqrt(underroot)
                    trustedFix.position_covariance[0] = out_cov
                    trustedFix.position_covariance[4] = out_cov
                    trustedFix.position_covariance[8] = 0
                fixCount += 1
                
                if started:
                    print ("Staaaaaarting... fix")
                    started=0
                if publish_gps:
                    print("trustedFix_pub") 
                    self.trustedFix_pub.publish(trustedFix)

            elif gps_qual == 4 :
                gpsQuality = 3
                badFlag += 5                
                out_cov = 3*math.sqrt(underroot)
                unTrustedFix.position_covariance[0] = out_cov 
                unTrustedFix.position_covariance[4] = out_cov
                unTrustedFix.position_covariance[8] = 0
                trustedCount = 0
                fixCount = 0
                if started:
                    print ("Staaaaaarting...fix")
                    started=0
                if publish_gps:
                    print("unTrustedFix_pub") 
                    self.unTrustedFix_pub.publish(unTrustedFix)

            elif gps_qual == 5 and lat_error < 0.1 and lon_error < 0.1:   
                gpsQuality = 5
                fixCount = 0           
                if started:
                    print ("Staaaaaarting... float")
                    started=0

                out_cov = 3*math.sqrt(underroot)
                trustedFloat.position_covariance[0] = out_cov
                trustedFloat.position_covariance[4] = out_cov
                trustedFloat.position_covariance[8] = 0
                trustedCount += 1
                if publish_gps:
                    self.trustedFloat_pub.publish(trustedFloat)
                    print("trustedFloat_pub") 
                    #trustedFloat.position_covariance[0] = 1.0
                    #trustedFloat.position_covariance[4] = 1.0
                    #trustedFloat.position_covariance[8] = 0 
                    #self.trustedFloat2_pub.publish(trustedFloat)
                                                            
            elif gps_qual == 5 and lat_error < 0.3 and lon_error < 0.3:  
                gpsQuality = 6
                fixCount = 0            
                if started:
                    print ("Staaaaaarting... float")
                    started=0
                out_cov = 3*math.sqrt(underroot)
                semiTrustedFloat.position_covariance[0] = out_cov
                semiTrustedFloat.position_covariance[4] = out_cov
                semiTrustedFloat.position_covariance[8] = 0
                trustedCount = 0
                if publish_gps:
                    print("semiTrustedFloat_pub") 
                    self.semiTrustedFloat_pub.publish(semiTrustedFloat)

            elif gps_qual == 5 :   
                gpsQuality = 7
                fixCount = 0
                badFlag += 5                            
                if started:
                    print ("Staaaaaarting... float")
                    started=0
                out_cov = 3*math.sqrt(underroot)
                unTrustedFloat.position_covariance[0] = out_cov
                unTrustedFloat.position_covariance[4] = out_cov
                unTrustedFloat.position_covariance[8] = 0
                trustedCount = 0
                if publish_gps:
                    print("unTrustedFloat_pub") 
                    self.unTrustedFloat_pub.publish(unTrustedFloat)


            else: 
                gpsQuality = 2
                fixCount = 0
                badFlag += 5                            
                if started:
                    print ("Staaaaaarting... float")
                    started=0
                out_cov = 3*math.sqrt(underroot)
                unTrustedFloat.position_covariance[0] = out_cov
                unTrustedFloat.position_covariance[4] = out_cov
                unTrustedFloat.position_covariance[8] = 0
                trustedCount = 0
                if publish_gps:
                    print("unTrustedFloat_pub") 
                    self.unTrustedFloat_pub.publish(unTrustedFloat)

            if satNo < 5 or hdop_GPGSA  > 4 or pdop_GPGSA > 8 :
                print (" **********************************************************************")
                print ("satNo , hdop_GPGSA , pdop_GPGSA ",satNo , hdop_GPGSA , pdop_GPGSA)
                print (" Very BAD GPS *********************************************************")
                badFlag += 5
                #os.system('spd-say "Warning, GPS is very bad" -t male1 -i -40 -r -10')
            elif satNo < 5 or hdop_GPGSA  > 1.8 or pdop_GPGSA > 5 :
                print( " :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::")
                print ("satNo , hdop_GPGSA , pdop_GPGSA ",satNo , hdop_GPGSA , pdop_GPGSA)
                badFlag += 1
                print (" BAD GPS :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::")
                #os.system('spd-say "Warning, GPS is bad" -t male1 -i -40 -r -10')
            elif satNo > 6 and hdop_GPGSA  < 1.7 and pdop_GPGSA < 4.3 and lat_error < 0.09 and lon_error < 0.09 and badFlag>0 :
                badFlag -= 1
           
            if trustedCount > 40 and badFlag > 0:
                badFlag = 0
                print (" resetting badFlag")

            '''
            allMsg.pose.covariance= [lat_error,lon_error,hdop_GGA,hdop_GPGSA,pdop_GPGSA,vdop_GPGSA,
                                    GPGSA_fix,SNR1[0],SNR2[0],SNR3[0],SNR4[0],SNR1[1],
                                    SNR2[1],SNR3[1],SNR4[1],SNR1[2],SNR2[2],SNR3[2],
                                    SNR4[2],SNR1[3],SNR2[3],SNR3[3],SNR4[3],satNo,
                                    hdop_GLGSA,pdop_GLGSA,vdop_GLGSA,hdop_GAGSA,pdop_GAGSA,vdop_GAGSA,
                                    gst_rms,utc_time,out_cov,gps_qual,gpsQuality, age_of_diff]
            allMsg.twist.twist.linear.x = kph_speed
            self.allMsg_pub.publish(allMsg)
            '''


    """Helper method for getting the frame_id with the correct TF prefix"""
    def get_frame_id(self):
        frame_id = self.declare_parameter('frame_id', 'gps_link').value
        prefix = self.declare_parameter('tf_prefix', '').value
        if len(prefix):
            return '%s/%s' % (prefix, frame_id)
        return frame_id
