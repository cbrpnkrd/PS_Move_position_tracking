# PS_Move_position_tracking
PS Move position tracking with Leap Motion controller for Unity engine.

PS Move orb was covered in retroreflective tape (3M 7610).
Image processing (threshold/contours/center of enclosing circle) was done through OpenCV code, compiled as dynamic library, and then accessed as native plugin from Unity engine.
Position was obtained through triangulation as described in Leap Motion documentation by using two circle centers as common feature in stereo pair.
Orientation received from UniMove plugin.

Video: https://youtu.be/qU9AcmfINHE
