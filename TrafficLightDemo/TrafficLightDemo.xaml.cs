using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text.RegularExpressions;
using System.Windows;
using System.Windows.Media;

using Microsoft.Kinect;

namespace TrafficLightDemo
{
    public partial class TrafficLightDemo : Window
    {
        #region Body frame display variables
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;
        #endregion

        const string HELP_TEXT = @"";
        const int BAUD_RATE = 9600;

        BodyFrameReader bodyFrameReader;
        KinectSensor sensor;
        SerialPort trafficLightSerialPort;
        bool trafficLightEn;

        public TrafficLightDemo()
        {
            trafficLightEn = false;
            sensor = KinectSensor.GetDefault();
            bodyFrameReader = sensor.BodyFrameSource.OpenReader();

            // initialize all the drawing variables
            InitializeBodyFrameDisplayVariables();

            bodyFrameReader.FrameArrived += BodyFrameArrived;
            sensor.Open();

            InitializeComponent();
        }

        public ImageSource ImageSource
        {
            get { return imageSource; }
        }

        private void TrafficLightSetPortButton_Click(object sender, RoutedEventArgs eventArgs)
        {
            string port = TrafficLightPortTextBox.Text;

            if (!Regex.Match(port, @"(?i)COM\d").Success)
            {
                MessageBox.Show("Invalid serial port - should be in the form COM#!", "Error");
                return;
            }

            trafficLightSerialPort = new SerialPort(port, BAUD_RATE);
            try 
            {
                trafficLightSerialPort.Open();
            }
            catch (Exception e)
            {
                MessageBox.Show(
                    string.Format(@"Failed to open {0}, please check that the traffic light is connected to that port. 
                                    Exception: {1}", port, e.GetType().Name),
                    "Error");
                return;
            }

            // successful opening - disable button & textbox
            TrafficLightPortTextBox.IsEnabled = false;
            TrafficLightSetPortButton.IsEnabled = false;
            trafficLightEn = true;
        }

        private void InitializeBodyFrameDisplayVariables()
        {
            // get the coordinate mapper
            coordinateMapper = sensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = sensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            displayWidth = frameDescription.Width;
            displayHeight = frameDescription.Height;

            // open the reader for the body frames
            bodyFrameReader = sensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            bones = new List<Tuple<JointType, JointType>>(
                new Tuple<JointType, JointType>[] {
                    // torso
                    new Tuple<JointType, JointType>(JointType.Head, JointType.Neck),
                    new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder),
                    new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid),
                    new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase),
                    new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight),
                    new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft),
                    new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight),
                    new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft),

                    // right arm
                    new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight),
                    new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight),
                    new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight),
                    new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight),
                    new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight),

                    // Left Arm
                    new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft),
                    new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft),
                    new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft),
                    new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft),
                    new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft),

                    // Right Leg
                    new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight),
                    new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight),
                    new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight),

                    // Left Leg
                    new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft),
                    new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft),
                    new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft)
                }
            );

            // populate body colors, one for each BodyIndex
            bodyColors = new List<Pen>();

            bodyColors.Add(new Pen(Brushes.Red, 6));
            bodyColors.Add(new Pen(Brushes.Orange, 6));
            bodyColors.Add(new Pen(Brushes.Green, 6));
            bodyColors.Add(new Pen(Brushes.Blue, 6));
            bodyColors.Add(new Pen(Brushes.Indigo, 6));
            bodyColors.Add(new Pen(Brushes.Violet, 6));

            // Create the drawing group we'll use for drawing
            drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            imageSource = new DrawingImage(drawingGroup);

            // use the window object as the view model in this simple example
            DataContext = this;
        }

        /// <summary>
        /// The traffic light has an arduino that listens for 
        /// the number of people tracked, by converting that number
        /// into an ASCII character, starting at 'a'. 
        /// The pattern is: 0 - red, slow; 1 - red, fast;
        /// 2 - yellow, slow .... 5 - green, fast; 6 - randomly
        /// cycle through all colors, quickly.
        /// </summary>
        /// <param name="peopleTracked"></param>
        private void UpdateTrafficLight(int peopleTracked)
        {
            char c = (char)(peopleTracked + 'a');
            trafficLightSerialPort.Write(char.ToString(c));
        }

        private void BodyFrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame == null)
                    return;

                if (bodies == null)
                    bodies = new Body[bodyFrame.BodyCount];

                // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                // As long as those body objects are not disposed and not set to null in the array,
                // those body objects will be re-used.
                bodyFrame.GetAndRefreshBodyData(bodies);

                // update the traffic light & GUI text
                int peopleTracked = bodies.Where(body => body.IsTracked || body.IsRestricted).Count();
                if (trafficLightEn)
                    UpdateTrafficLight(peopleTracked);
                PeopleTrackedLabel.Content = string.Format("# of people tracked: {0}", peopleTracked);
            }

            using (DrawingContext dc = drawingGroup.Open())
            {
                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                int penIndex = 0;
                foreach (Body body in this.bodies)
                {
                    Pen drawPen = this.bodyColors[penIndex++];

                    if (!body.IsTracked)
                        continue;

                    DrawClippedEdges(body, dc);

                    IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                    // convert the joint points to depth (display) space
                    Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                    foreach (JointType jointType in joints.Keys)
                    {
                        // sometimes the depth(Z) of an inferred joint may show as negative
                        // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                        CameraSpacePoint position = joints[jointType].Position;
                        if (position.Z < 0)
                            position.Z = InferredZPositionClamp;

                        DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                        jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                    }

                    DrawBody(joints, jointPoints, dc, drawPen);
                    DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                    DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                }

                // prevent drawing outside of our render area
                drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, displayWidth, displayHeight));
            }
        }

        public void Dispose()
        {
            if (bodyFrameReader != null)
                bodyFrameReader.Dispose();

            if (sensor != null && sensor.IsOpen)
                sensor.Close();
        }

        #region Drawing code
        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }
        #endregion
    }
}
