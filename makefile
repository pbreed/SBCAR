#    Copyright 1998-2012 NetBurner, Inc.  ALL RIGHTS RESERVED
#    Permission is hereby granted to purchasers of NetBurner Hardware
#    to use or modify this computer program for any use as long as the
#    resultant program is only executed on NetBurner provided hardware.
# 
#    No other rights to use this program or it's derivitives in part or
#    in whole are granted.
# 
#    It may be possible to license this or other NetBurner software for
#    use on non NetBurner Hardware. Please contact Licensing@Netburner.com 
#    for more information.
# 
#    NetBurner makes no representation or warranties with respect to the 
#    performance of this computer program, and specifically disclaims any 
#    responsibility for any damages, special or consequential, connected 
#    with the use of this program.
# 
#    NetBurner, Inc
#    http://www.netburner.com

NAME = SBEXACA
PLATFORM = SBL2EXA

CXXSRCS = \
	main.cpp imu.cpp servo.cpp dsm2_sub.cpp sensor_config.cpp pitr_sem.cpp log.cpp gps.cpp checkstack.cpp wp.cpp ned.cpp modsmart.cpp

CSRCS = \

include $(NBROOTMINGW)/make/main.mak




