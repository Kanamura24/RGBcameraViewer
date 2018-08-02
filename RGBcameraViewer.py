#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file RGBcameraViewer.py
 @brief RGB camera image viewer 
 @date $Date$


"""
import sys
import time
sys.path.append(".")

from threading import Lock
lock = Lock()

# capture
import numpy as np
from numpy import array
import cv2

# Import RTM module
import RTC
import OpenRTM_aist



cameraImageQueue = []

# Import Service implementation class
# <rtc-template block="service_impl">

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
rgbcameraviewer_spec = ["implementation_id", "RGBcameraViewer", 
		 "type_name",         "RGBcameraViewer", 
		 "description",       "RGB camera image viewer ", 
		 "version",           "1.0.0", 
		 "vendor",            "VenderName", 
		 "category",          "Category", 
		 "activity_type",     "STATIC", 
		 "max_instance",      "1", 
		 "language",          "Python", 
		 "lang_type",         "SCRIPT",
		 ""]
# </rtc-template>

##
# @class RGBcameraViewer
# @brief RGB camera image viewer 
# 
# 
class RGBcameraViewer(OpenRTM_aist.DataFlowComponentBase):
	
	##
	# @brief constructor
	# @param manager Maneger Object
	# 
	def __init__(self, manager):
		OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

		self._d_RGBcameraImage = RTC.CameraImage(RTC.Time(0,0),0,0,0,'',1.0,[])
		"""
		"""
		self._RGBcameraImageIn = OpenRTM_aist.InPort("RGBcameraImage", self._d_RGBcameraImage)

                
                self._d_checkPoint = RTC.TimedBoolean(RTC.Time(0,0),0)
		"""
		"""
		self._checkPointIn = OpenRTM_aist.InPort("checkPoint", self._d_checkPoint)

		


		# initialize of configuration-data.
		# <rtc-template block="init_conf_param">
		
		# </rtc-template>


		 
	##
	#
	# The initialize action (on CREATED->ALIVE transition)
	# formaer rtc_init_entry() 
	# 
	# @return RTC::ReturnCode_t
	# 
	#
	def onInitialize(self):
		# Bind variables and configuration variable
		
		# Set InPort buffers
		self.addInPort("RGBcameraImage",self._RGBcameraImageIn)
                self.addInPort("checkPoint",self._checkPointIn)
		
		# Set OutPort buffers
		
		# Set service provider to Ports
		
		# Set service consumers to Ports
		
		# Set CORBA Service Ports
		
		return RTC.RTC_OK
	
	#	##
	#	# 
	#	# The finalize action (on ALIVE->END transition)
	#	# formaer rtc_exiting_entry()
	#	# 
	#	# @return RTC::ReturnCode_t
	#
	#	# 
	#def onFinalize(self):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The startup action when ExecutionContext startup
	#	# former rtc_starting_entry()
	#	# 
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onStartup(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The shutdown action when ExecutionContext stop
	#	# former rtc_stopping_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onShutdown(self, ec_id):
	#
	#	return RTC.RTC_OK
	
		##
		#
		# The activated action (Active state entry action)
		# former rtc_active_entry()
		#
		# @param ec_id target ExecutionContext Id
		# 
		# @return RTC::ReturnCode_t
		#
		#
	def onActivated(self, ec_id):
                print('Start image view')

                #cv2.namedWindow("img", cv2.WINDOW_AUTOSIZE)

                self._img = np.zeros((768, 1024, 3), np.uint8)
                self._storedPoint = None
	
		return RTC.RTC_OK
	
		##
		#
		# The deactivated action (Active state exit action)
		# former rtc_active_exit()
		#
		# @param ec_id target ExecutionContext Id
		#
		# @return RTC::ReturnCode_t
		#
		#
	def onDeactivated(self, ec_id):

                
                cv2.destroyAllWindows()
                print("Stop image view.")
                
		return RTC.RTC_OK
	
		##
		#
		# The execution action that is invoked periodically
		# former rtc_active_do()
		#
		# @param ec_id target ExecutionContext Id
		#
		# @return RTC::ReturnCode_t
		#
		#
	def onExecute(self, ec_id):

                 #Timer
                if self._checkPointIn.isNew():
                        inJudge = self._checkPointIn.read()

                        self._storedPoint = inJudge.data;


                        global pointCount
                        global textColor
                        
                        if self._storedPoint == 1:
                                pointCount = 'o'
                                textColor = 255, 0, 0
                                
                        else:
                                pointCount = 'x'
                                textColor = 0, 0, 255
                        
                        
                        

                #cameraImage
                
                if self._RGBcameraImageIn.isNew():
                        img = self._RGBcameraImageIn.read()
                        global cameraImageQueue
                        lock.acquire()
                        # cameraImage = np.zeros((480, 640, 3), np.uint8)
                        cameraImage = np.fromstring(img.pixels, dtype = np.uint8).reshape(img.height, img.width, -1)

                        if not self._storedPoint == None:
                                                                
                                cv2.putText(cameraImage,
                                            #'%d' % (pointCount),
                                            '%s' % (pointCount),
                                            (0,50),
                                            cv2.FONT_HERSHEY_COMPLEX, 1, (textColor), 3, cv2.LINE_AA)
                        cameraImageQueue.append(cameraImage)
                        lock.release()

                        # cv2.imshow('image', cvimage)
                        # cv2.waitKey(-1)


              


                        
		return RTC.RTC_OK
	
	#	##
	#	#
	#	# The aborting action wh;en main logic error occurred.
	#	# former rtc_aborting_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onAborting(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The error action in ERROR state
	#	# former rtc_error_do()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onError(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The reset action that is invoked resetting
	#	# This is same but different the former rtc_init_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onReset(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The state update action that is invoked after onExecute() action
	#	# no corresponding operation exists in OpenRTm-aist-0.2.0
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#

	#	#
	#def onStateUpdate(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The action that is invoked when execution context's rate is changed
	#	# no corresponding operation exists in OpenRTm-aist-0.2.0
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onRateChanged(selpf, ec_id):
	#
	#	return RTC.RTC_OK
	



def RGBcameraViewerInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=rgbcameraviewer_spec)
    manager.registerFactory(profile,
                            RGBcameraViewer,
                            OpenRTM_aist.Delete)

def MyModuleInit(manager):
    RGBcameraViewerInit(manager)

    # Create a component
    comp = manager.createComponent("RGBcameraViewer")


def showImage():
        key = ''
        lock.acquire()
        #print 'show'
        if len(cameraImageQueue) > 0:
                #print 'Image'
                cameraImage = cameraImageQueue.pop()
                cv2.imshow('image', cameraImage)
                key = cv2.waitKey(1)
                
        lock.release()

        if key == 'q':
                return True
        return False
def main():
	mgr = OpenRTM_aist.Manager.init(sys.argv)
	mgr.setModuleInitProc(MyModuleInit)
	mgr.activateManager()
	mgr.runManager(True)


        while True:
                time.sleep(0.1)
                if showImage() == True:
                        return


if __name__ == "__main__":
	main()

        
