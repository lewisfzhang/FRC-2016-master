import os

class C290Ctl:
  def __init__(self, id):
    self.id = id

  def __setControl(self, ctl, value):
    print "v4l2-ctl -d /dev/video %d -c %s=%s" % (self.id, ctl, value)
    # Eventually run this on the system?

  def setAutoExposure(self, on):
    self.__setControl("auto_exposure", "1" if on else "0")
    return on

  def setManualExposure(self, exposure):
    self.setAutoExposure(False)
    self.__setControl("exposure", exposure)
    return exposure
