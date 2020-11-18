# The_NMEAtor

Update November 18, 2020; Please use the Yazz_NMEAtor repository which contains the lastest version
 
Introduction:

I've an old Robertson data network on board feeding the on board devices with NMEA0183 v1.5. Not only is this RS-232 related, but the NMEA sentences are not terminated with a checksum. Most navigation apps don't care and checksum checking can be overridden by the user. There is how ever a newer NMEA0183 v2.0 related data stream for GPS and AIS information that needs to be integrated with the old data stream, better known as multiplexing.
