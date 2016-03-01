using UnityEngine;
using System;
using System.Threading;
using System.Runtime.InteropServices;
using Leap;

public class PluginCaller : MonoBehaviour
{
    public Transform handController; //Leap Motion hand controller for coordinate conversion 
    public struct ReturnPoint
    {
        public int x;
        public int y;
    };

    [DllImport("CVTrack")]
    private static extern bool GetTrackedPoint(System.IntPtr inArr, int h, int w, ref ReturnPoint result);

    Controller controller;
    GCHandle inHandleLeft, inHandleRight;
    ReturnPoint resLeft, resRight;
    Image imageLeft, imageRight;
    Frame frame;
    Vector slopes_left, slopes_right;
    float cameraZ, cameraY, cameraX;
    Vector pntLeft, pntRight;
    Vector3 prevPos, trPos;
    bool isAllocated = false;
    float scale = 1000.0f; //from mm to m
    bool isLeftPntFound = false;
    bool isRightPntFound = false;
    bool isTracked = false; //tracking state, for single actions on tacking lost/found

    // Use this for initialization
    void Start()
    {
        //leap motion
        controller = new Controller();
        controller.SetPolicy(Controller.PolicyFlag.POLICYBACKGROUNDFRAMES);
        controller.SetPolicy(Controller.PolicyFlag.POLICY_IMAGES);
        pntLeft = new Vector(0, 0, 0);
        pntRight = new Vector(0, 0, 0);
        prevPos = new Vector3(0, 0, 0);
    }

    void OnDestroy()
    {
        inHandleLeft.Free();
        inHandleRight.Free();
    }

    // Update is called once per frame
    void Update()
    {
        if (controller.IsConnected)
        {
            //get data fram from device
            frame = controller.Frame();

            if (frame.IsValid)
            {
                imageLeft = frame.Images[0];
                imageRight = frame.Images[1];

                //allocate for the first time
                if (!isAllocated) //once
                {
                    if (imageLeft.Data.Length > 0 && imageRight.Data.Length > 0)
                    {
                        inHandleLeft = GCHandle.Alloc(imageLeft.Data, GCHandleType.Pinned);
                        inHandleRight = GCHandle.Alloc(imageRight.Data, GCHandleType.Pinned);
                        isAllocated = true;
                    }
                }
                else
                {
                    //update
                    inHandleLeft.Target = imageLeft.Data;
                    inHandleRight.Target = imageRight.Data;
                }

                //left image feature tracking                
                isLeftPntFound = GetTrackedPoint(inHandleLeft.AddrOfPinnedObject(), imageLeft.Height, imageLeft.Width, ref resLeft);

                //right image feature tracking
                isRightPntFound = GetTrackedPoint(inHandleRight.AddrOfPinnedObject(), imageRight.Height, imageRight.Width, ref resRight);

                if (isLeftPntFound && isRightPntFound) //feature point found in both images
                {
                    if (!isTracked)
                    {
                        Debug.Log("Tracking acquired!");
                        isTracked = true;
                    }

                    //set feature vectors
                    pntLeft.x = resLeft.x;
                    pntLeft.y = resLeft.y;
                    pntRight.x = resRight.x;
                    pntRight.y = resRight.y;

                    //triangulation
                    slopes_left = imageLeft.Rectify(pntLeft);
                    slopes_right = imageRight.Rectify(pntRight);

                    //Do the triangulation from the rectify() slopes
                    cameraZ = 40 / (slopes_right.x - slopes_left.x);
                    cameraY = cameraZ * slopes_right.y;
                    cameraX = cameraZ * slopes_right.x - 20;

                    //set resulting vector values
                    trPos.x = cameraX;
                    trPos.z = cameraY;
                    trPos.y = -cameraZ;

                    gameObject.transform.position = handController.TransformPoint((prevPos + trPos) / (2 * scale)); //scale + averaging on previous position; from Leap to world coordinated
                    prevPos = trPos;
                }
                else //feature not found
                {
                    if (isTracked) //once
                    {
                        Debug.Log("Tracking lost");
                        isTracked = false;
                    }

                }
            }
        }
    }
}