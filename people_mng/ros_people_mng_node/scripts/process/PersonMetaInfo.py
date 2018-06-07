import rospy


class PersonMetaInfo():
    SHIRT_RECT="SHIRT_RECT"
    TROUSER_RECT="TROUSER_RECT"
    PERSON_RECT="PERSON_RECT"

    id=-1
    posture=''
    handCall=False
    _boundingBoxMap={}
    _colorNameMap={}
    _colorRGBMap={}
    distanceEval=0
    label_id=''
    
    def __init__(self,id):
        self.id=id

    def setBoundingBox(self,nameKey,ptList):
        try:
            self._boundingBoxMap[nameKey]=ptList
        except Exception as e:
            rospy.logwarn("Unable to add bounding box:"+str(e))

    def setMainColor(self,nameKey,color_label,rgb_value):
        try:
            self._colorNameMap[nameKey]=color_label
            self._colorRGBMap[nameKey]=rgb_value
        except Exception as e:
            rospy.logwarn("Unable to add color:"+str(e))

    def getMainColor(self,nameKey):
        value='NONE'
        try:
            value=self._colorNameMap[nameKey]
            
        except Exception as e:
            rospy.logwarn("Unable to get color:"+str(e))
        return value

    def __str__(self):
        return '{"boundingBoxMap":'+str(self._boundingBoxMap)+'",colorNameMap":'+str(self._colorNameMap)+'",colorRGBMap":'+str(self._colorRGBMap)+'",distanceEval":'+str(self.distanceEval)+'",label_id":'+str(self.label_id)+'",posture":'+str(self.posture)+'",handCall":'+str(self.handCall)+'",id":'+str(self.id)+'}'

