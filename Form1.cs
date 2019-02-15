using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
//using PylonC.NET;
using Basler.Pylon;
using System.Net;
using System.Net.Sockets;
using OpenCvSharp;
using OpenCvSharp.Extensions;
using EasyModbus;
using System.Drawing.Imaging;
using System.Threading;
using DALSA.SaperaProcessing.CPro;
using DALSA.SaperaProcessing.CProSearch;
using DALSA.SaperaProcessing.CProCommonDemo;
using DALSA.SaperaProcessing.CProMeasurement;
using System.Runtime.InteropServices;
using System.Collections;
namespace Brakes_India
{
    
    public partial class Brakes_India : Form
    {
        
        public static string passtext;
        public static string CAMDET;


        public static Mat img1 = new Mat();
        public static bool[] m = new bool[33];
        
        private const float RADIAN_PER_DEGREE = 0.0174532f;
        private string m_ImagePath;
        private float m_ExecuteTime1;
        private float m_ExecuteTime2;
        private CDemoTimer m_Timer;
        private CProImage Image;
        private CProMeasMarkerSpoke spoke1;
        private CProMeasMarkerSpoke spoke2;
        private CProMeasMarkerRake rake1;
        private CProMeasMarkerRake rake2;
        private CProHoughLine l1;
        private CProHoughLine l2;
        private CProHoughLine l3;
        private CProHoughLine l4;
        private CProHoughLine shdl1;
        private CProHoughLine shdl2;
        private CProHoughLine shdl3;
        private CProHoughLine shdl4;
        private CProMeasBasic perRline;
        private CProMeasBasic perLline;
        private CProMeasBasic segRline;
        private CProMeasBasic segLline;
        private CProMeasBasic intersectRline;
        private CProMeasBasic intersectLline;
        private CProMeasBasic distance;
        CProImage shdmeasure = new CProImage();
        CProImage shdroi = new CProImage();
        private Dictionary<string, CProRect> rectRois;
        private Dictionary<string, CProSearchEdge> edgeSearches;
        private Dictionary<string, CProSearchArea> areaSearches;
        private Dictionary<string, CProHoughCircle> HoughCircles;
        private CProBasic preproc;
        private bool[] enableEqualize;
        private float[] srcLow;
        private float[] srcHigh;
        private float[] dstLow;
        private float[] dstHigh;
        private float[] threshlo;
        private float[] threshhi;
        private bool[] threshinv;
        private bool[] enableHistEqlze;
        private bool[] enableThreshold;
        private bool[] enableSearchEdge;
        private bool[] enableSearchArea;
        private bool[] enableEdgeDetect;
        private int[] edgeThresh;
        private bool[] enableHoughCircle;
        private bool[] pass_fail;
        string n;string foldername= DateTime.Now.ToString("yyyy-MM-dd");
        string[] counting = new string[50];
        string name;
        public Brakes_India()
        {
            Image = new CProImage();
            spoke2 = new CProMeasMarkerSpoke();
            spoke1 = new CProMeasMarkerSpoke();
            rake1 = new CProMeasMarkerRake();
            rake2 = new CProMeasMarkerRake();
            l1 = new CProHoughLine();
            l2 = new CProHoughLine();
            l3 = new CProHoughLine();
            l4 = new CProHoughLine();
            shdl1 = new CProHoughLine();
            shdl2 = new CProHoughLine();
            shdl3 = new CProHoughLine();
            shdl4 = new CProHoughLine();
            perRline = new CProMeasBasic();
            perLline = new CProMeasBasic();
            segRline = new CProMeasBasic();
            segLline = new CProMeasBasic();
            intersectRline = new CProMeasBasic();
            intersectLline = new CProMeasBasic();
            distance = new CProMeasBasic();
            m_Timer = new CDemoTimer();
            edgeSearches = new Dictionary<string, CProSearchEdge>();
            areaSearches = new Dictionary<string, CProSearchArea>();
            HoughCircles = new Dictionary<string, CProHoughCircle>();
            rectRois = new Dictionary<string, CProRect>();
            preproc = new CProBasic();
            
            InitializeComponent();
        }
        Camera usbcam;
        private static readonly log4net.ILog log =
            log4net.LogManager.GetLogger(System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);
       //PYLON_DEVICE_HANDLE hDev = new PYLON_DEVICE_HANDLE(); /* Handle for the pylon device. */
        // uint numDevices;    /* Number of available devices. */
        const int numGrabs = 10; /* Number of images to grab. */
       // PylonBuffer<Byte> imgBuf = null;  /* Buffer used for grabbing. */
        bool isAvail;
        int i = 0;
        TcpClient tcpclnt = new TcpClient();
        ModbusClient iomodule;
        Stream stm;
        byte[] bb = new byte[100];
        byte[] ba = new byte[100];
        bool btnrun = false;
        bool sftTrig = true;
        bool extTrig = false;
        bool shEnble = false;
        Mat templl = new Mat();
        Mat templr = new Mat();
        Mat templ1 = new Mat();
        Mat templ2 = new Mat();
        Mat templ3 = new Mat();
        Mat templ4 = new Mat();
        Mat templ5 = new Mat();
        Mat templ6 = new Mat();
        Mat templ7 = new Mat();
        Mat templ8 = new Mat();
        int nonzpix1 = 100;
        int plus_dis1 = 400;
        int red_clrs1 = 1;
        int nonzpix2 = 100;
        int plus_dis2 = 100;
        int red_clrs2 = 2;
        int nonzpix = 100;
        int plus_dis = 0;
        int red_clrs = 0;
        double expo_600 = 10000;
        double expo_1100 = 10000;
        double cam_expo = 10000;
        int delay_600 = 200;
        int delay_1100 = 200;
        int trig_delay = 0;
        Rectangle rect;
        OpenCvSharp.Point StartLocation;
        OpenCvSharp.Point EndLcation;
        bool IsMouseDown = false;
        Mat imgInput = new Mat();
        int temp_no = 0;
        int pass = 0, fail = 0, tot = 0;
        public static int passed = 0, failed = 0, total = 0;
        public static double lw, lh, rw, rh;
        public static double pinposition;
        public static double pincenter, addroi;
        static Version Sfnc2_0_0 = new Version(2, 0, 0);
        // The exit code of the sample application.
        int exitCode = 0;
        PixelDataConverter converter = new PixelDataConverter();
        bool sl_loaded = false;
        bool img_save = true;
        bool draw = false;
        bool draw_roi = false;
        int Rm = 80, Gm = 40, Bm = 30, Rx = 150, Gx = 90, Bx = 80;
        int enblAdmin = 0;
        bool trig1 = false, trig2= false;
        private bool drawres;
        int COUNTS = 0;
        
        string de1 = ",";
        double getsubrpix, getrpix, getsublpix, getlpix = 0;
        float r = 0;
        CProFPoint centerPoint;
        int pindist;
        Mat img, grayimg = new Mat();


        private void basler_gige_grab_Load(object sender, EventArgs e)
        {
            try
            {
                img_disp.Width = 2590;
                img_disp.Height = 1942;
                panel1.Controls.Add(img_disp);
                panel1.AutoScroll = true;
                this.Controls.Add(panel1);
               if (shEnble)
               {
                    txtBox.Text += "Connecting.....\r\n";
                    tcpclnt.Connect("192.168.0.100", 1024);
                    // use the ipaddress as in the server program

                    txtBox.Text += "Connected \r\n";
                    stm = tcpclnt.GetStream();
               }
                log.Info("started..");
                load_settings();
                // Ask the camera finder for a list of camera devices.
                List<ICameraInfo> allCameras = CameraFinder.Enumerate();
                if (allCameras.Count==0)
                {
                    tlStrpPrgsBar1.Value = 100;
                    throw new Exception("No devices found.");
                }
                usbcam = new Camera();
                iomodule = new ModbusClient("COM4");
                iomodule.Parity = System.IO.Ports.Parity.Even;
                iomodule.Connect();
                timer2.Enabled = true;

                // Before accessing camera device parameters, the camera must be opened.
                usbcam.Open();
                tlStrpPrgsBar1.Value = 20;
                // Set an enum parameter.
                string oldPixelFormat = usbcam.Parameters[PLCamera.PixelFormat].GetValue(); // Remember the current pixel format.
                Console.WriteLine("Old PixelFormat  : {0} ({1})", usbcam.Parameters[PLCamera.PixelFormat].GetValue(), oldPixelFormat);
                Console.WriteLine("count  : {0}", usbcam.Parameters[PLCamera.AcquisitionMode].GetValue());
                if (!usbcam.Parameters[PLCamera.PixelFormat].TrySetValue(PLCamera.PixelFormat.Mono8))
                {
                    /* Feature is not available. */
                    throw new Exception("Device doesn't support the Mono8 pixel format.");
                }

                tlStrpPrgsBar1.Value = 50;
                usbcam.Parameters[PLCamera.TriggerMode].TrySetValue(PLCamera.TriggerMode.Off);
                if (extTrig)
                {
                    usbcam.Parameters[PLCamera.TriggerSource].TrySetValue(PLCamera.TriggerSource.Line1);
                    usbcam.Parameters[PLCamera.TriggerMode].TrySetValue(PLCamera.TriggerMode.On);
                }
                //usbcam.Parameters[PLCamera.ExposureTime].TrySetValue(10000);
                usbcam.StreamGrabber.ImageGrabbed += OnImageGrabbed;
                usbcam.StreamGrabber.GrabStopped += OnGrabStopped;
                tlStrpPrgsBar1.Value = 100;
                tlStrpStsLbl1.Text = "Camera connected..";
                CAMDET = tlStrpStsLbl1.Text;
                tlStrpStsLbl1.Image = global::Brakes_India.Properties.Resources.ok;
                if(sftTrig | extTrig)
                    btnStart.Enabled = true;
                ckBox_SH.Enabled = true;
                ckBox_ST.Enabled = true;
            }
            catch (Exception ex)
            {
                tlStrpPrgsBar1.Value = 100;
                tlStrpStsLbl1.Image = global::Brakes_India.Properties.Resources.error;
                tlStrpStsLbl1.Text = "Error..... " + ex.StackTrace;
               
                /* Retrieve the error message. */
                tlStrpStsLbl1.Text = "Exception caught:";
                tlStrpStsLbl1.Text += ex.Message;
                if (ex.Message != " ")
                {
                    //tlStrpStsLbl1.Text += "Last error message:" + ex.Message;
                    //tlStrpStsLbl1.Text += msg;
                }
                try
                {
                    // Close the camera.
                    if (usbcam != null)
                    {
                        usbcam.Close();
                        usbcam.Dispose();
                    }
                }
                catch (Exception)
                {
                    /*No further handling here.*/
                }

               // Pylon.Terminate();  /* Releases all pylon resources. */
                btnStart.Enabled = false;
                ckBox_SH.Enabled = false;
                ckBox_ST.Enabled = false;
                ckBox_ET.Enabled = false;
                btnIspectOnce.Enabled = false;
                txtBox.Text += "\nPress enter to exit.\r\n";
            }
        }
        // Occurs when an image has been acquired and is ready to be processed.
        private void OnImageGrabbed(Object sender, ImageGrabbedEventArgs e)
        {
            draw = false;
            draw_roi = false;
            if (InvokeRequired)
            {
                // If called from a different thread, we must use the Invoke method to marshal the call to the proper GUI thread.
                // The grab result will be disposed after the event call. Clone the event arguments for marshaling to the GUI thread.
                BeginInvoke(new EventHandler<ImageGrabbedEventArgs>(OnImageGrabbed), sender, e.Clone());
                return;
            }
            Thread.Sleep(200);
            //iomodule.WriteSingleCoil(1283, false);
            tlStrpPrgsBar1.Value = 20;
           
            if (shEnble)
            {

                    Byte min, max;
                  //  PylonGrabResult_t grabResult;
                    int k = stm.Read(bb, 0, 1);

                    SetText(Convert.ToString(bb));
                if ((int)bb[0] == 100 && btnrun == true)
                {
                    SetText("Grab Image\r\n");
                    /* Grab one single frame from stream channel 0. The
                    camera is set to "single frame" acquisition mode.
                    Wait up to 500 ms for the image to be grabbed.
                    If imgBuf is null a buffer is automatically created with the right size.*/
                    // IGrabResult res =  usbcam.StreamGrabber.GrabOne(10000000, TimeoutHandling.Return);
                    // Get the grab result.
                    IGrabResult res = e.GrabResult;
                    if (!res.IsValid)
                    {
                        /* Timeout occurred. */
                        SetText(String.Format("Frame {0}: timeout.\r\n", i + 1));
                    }

                    /* Check to see if the image was grabbed successfully. */
                    if (res.GrabSucceeded)
                    {
                        /* Success. Perform image processing. */
                        Bitmap bitmap = new Bitmap(res.Width, res.Height, PixelFormat.Format32bppRgb);
                        // Lock the bits of the bitmap.
                        BitmapData bmpData = bitmap.LockBits(new Rectangle(0, 0, bitmap.Width, bitmap.Height), ImageLockMode.ReadWrite, bitmap.PixelFormat);
                        // Place the pointer to the buffer of the bitmap.
                        converter.OutputPixelFormat = PixelType.BGRA8packed;
                        IntPtr ptrBmp = bmpData.Scan0;
                        converter.Convert(ptrBmp, bmpData.Stride * bitmap.Height, res); //Exception handling TODO
                        bitmap.UnlockBits(bmpData);
                        // img = new Mat();
                        img = BitmapConverter.ToMat(bitmap);
                        
                        //if (img_save)
                        //    Cv2.ImWrite(folderBrowserDialog1.SelectedPath + "\\"+i.ToString()+".bmp", img);
                        // Assign a temporary variable to dispose the bitmap after assigning the new bitmap to the display control.
                        Bitmap bitmapOld = img_disp.Image as Bitmap;
                        img_disp.Image = bitmap;
                        if (bitmapOld != null)
                        {
                            // Dispose the bitmap.
                            bitmapOld.Dispose();
                        }
                        Cv2.CvtColor(img, grayimg, ColorConversionCodes.BGR2GRAY);
                        Image.Set(bitmap.Width, bitmap.Height, CProData.FormatEnum.FormatColorUByte, grayimg.Data, true);
                        process_image_sapera(Image);
                    }
                    else if (!res.GrabSucceeded)
                    {
                        SetText(String.Format("Frame {0} wasn't grabbed successfully.  Error code = {1}\r\n", i + 1, res.ErrorCode));
                    }
                    Array.Clear(bb, 0, bb.Length);
                    ba = BitConverter.GetBytes(101);
                    stm.Write(ba, 0, 1);
                    SetText("Transmited.....\r\n");
                    Array.Clear(ba, 0, ba.Length);

                }
                else {
                    btnrun = false;
                    ckBox_SH.Enabled = true;
                    ckBox_ST.Enabled = true;
                    //ckBox_ET.Enabled = true;
                    btnStart.Enabled = true;
                    btnIspectOnce.Enabled = true;
                    try
                    {
                        usbcam.StreamGrabber.Stop();
                    }
                    catch (Exception exception)
                    {
                        throw (exception);
                    }

                }
                ++i;
            }
            else if(btnrun == true)
            {
                    Byte min, max;
                    //PylonGrabResult_t grabResult;
                    SetText("Grab Image\r\n");
                /* Grab one single frame from stream channel 0. The
                camera is set to "single frame" acquisition mode.
                Wait up to 500 ms for the image to be grabbed.
                If imgBuf is null a buffer is automatically created with the right size.*/
                //IGrabResult res = usbcam.StreamGrabber.GrabOne(10000000, TimeoutHandling.Return);
                IGrabResult res = e.GrabResult;
                if (!res.IsValid)
                {
                        /* Timeout occurred. */
                        SetText(String.Format("Frame {0}: timeout.\r\n", i + 1));
                }

                    /* Check to see if the image was grabbed successfully. */
                    if (res.GrabSucceeded)
                    {
                        /* Success. Perform image processing. */
                        Bitmap bitmap = new Bitmap(res.Width, res.Height, PixelFormat.Format32bppRgb);
                        // Lock the bits of the bitmap.
                        BitmapData bmpData = bitmap.LockBits(new Rectangle(0, 0, bitmap.Width, bitmap.Height), ImageLockMode.ReadWrite, bitmap.PixelFormat);
                        // Place the pointer to the buffer of the bitmap.
                        converter.OutputPixelFormat = PixelType.BGRA8packed;
                        IntPtr ptrBmp = bmpData.Scan0;
                        converter.Convert(ptrBmp, bmpData.Stride * bitmap.Height, res); //Exception handling TODO
                        bitmap.UnlockBits(bmpData);
                        // img = new Mat();
                        img = BitmapConverter.ToMat(bitmap);

                    // Assign a temporary variable to dispose the bitmap after assigning the new bitmap to the display control.
                    //if (img_save)
                    //    Cv2.ImWrite(folderBrowserDialog1.SelectedPath + "\\" + i.ToString() + ".bmp", img);
                    Bitmap bitmapOld = img_disp.Image as Bitmap;
                    img_disp.Image = bitmap;
                    img.CopyTo(img1);
                    if (bitmapOld != null)
                    {
                        // Dispose the bitmap.
                        bitmapOld.Dispose();
                    }
                    Cv2.CvtColor(img, grayimg, ColorConversionCodes.BGR2GRAY);
                    Image.Set(bitmap.Width, bitmap.Height, CProData.FormatEnum.FormatUByte, grayimg.Data, true);
                    process_image_sapera(Image);
                    }
                    else if (!res.GrabSucceeded)
                    {
                        SetText(String.Format("Frame {0} wasn't grabbed successfully.  Error code = {1}\r\n", i + 1, res.ErrorCode));
                    }
                ++i;
            }
            tlStrpPrgsBar1.Value = 100;
            tlStrpStsLbl1.Text = String.Format("Frame {0} grabbed.",i+1);
            // Dispose the grab result if needed for returning it to the grab loop.
            e.DisposeGrabResultIfClone();
        }
        // Occurs when a camera has stopped grabbing.
        private void OnGrabStopped(Object sender, GrabStopEventArgs e)
        {
            if (InvokeRequired)
            {
                // If called from a different thread, we must use the Invoke method to marshal the call to the proper thread.
                BeginInvoke(new EventHandler<GrabStopEventArgs>(OnGrabStopped), sender, e);
                return;
            }
            btnrun = false;
            ckBox_SH.Enabled = true;
            ckBox_ST.Enabled = true;
            btnStop.Enabled = true;
            btnStart.Enabled = true;
            btnIspectOnce.Enabled = true;
        }

        private void process_image_sapera(CProImage imgIn, bool offline = false)
        {
            tlStrpPrgsBar1.Value = 0;
            //Cursor = Cursors.WaitCursor;

            if (!Image.Valid)
                return;
            bool tot_result = true;
            bool passlead = true;
            bool passabut = true;
            bool passpaul = true;
            bool passrivet = true;
            bool passsizer = true;
            bool passsizel = true;
            bool passclr = true;
            CProImage shdmeasure = new CProImage();
            for (int i = 0; i < cmbBx_model.Items.Count; i++)
            {

               
                string cmbxItem = cmbBx_model.GetItemText(cmbBx_model.Items[i]);
                CProImage testimg = new CProImage(imgIn.Width, imgIn.Height, imgIn.Format, rectRois[cmbxItem], imgIn.GetData(), true);
                //if (cmbxItem == "SHDRRight" || cmbxItem == "SHDRLeft" || cmbxItem == "SHDRBottom")
                //{
                //    testimg = shdmeasure.Clone();

                //}
                if (enableHistEqlze[i])
                {
                    preproc.HistEqualize(testimg, testimg);
                }
                if (enableEqualize[i])
                {
                    preproc.Equalize(testimg, testimg, srcLow[i], srcHigh[i], dstLow[i], dstHigh[i]);
                }
                if (enableThreshold[i])
                {
                    preproc.Thresh(testimg, testimg, threshlo[i], threshhi[i], threshinv[i], CProBasic.ThreshLevelType.ThreshLevelDouble, CProBasic.ThreshResultType.ThreshResultBinary);
                }
                if (enableEdgeDetect[i])
                {
                    CProImage tempimg = new CProImage(testimg.Width, testimg.Height, testimg.Format, testimg.Roi, testimg.GetData(), true);
                    preproc.EdgeDetect(tempimg, testimg, CProBasic.EdgeDetectType.EdgeDetectPrewitt, edgeThresh[i]);
                }

                if (enableSearchEdge[i])
                {
                    // Remember default search parameters
                    int maxMatches = edgeSearches[cmbxItem].MaxMatches;
                    float minRotation = edgeSearches[cmbxItem].MinRotation;
                    float maxRotation = edgeSearches[cmbxItem].MaxRotation;
                    tlStrpPrgsBar1.Value = 40;
                    // Set search parameters
                    edgeSearches[cmbxItem].MinScore = (int)numericUpDown1.Value;
                    edgeSearches[cmbxItem].MaxMatches = 1;
                    //searches[i].CoarseResolution = 1;
                    //searches[i].FineResolution = 1;
                    //searches[i].AcceptScore = 30;
                    //searches[i].CandidateScore = 30;

                    edgeSearches[cmbxItem].MinRotation = -30;
                    edgeSearches[cmbxItem].MaxRotation = 30;
                    edgeSearches[cmbxItem].MinScore = 30;
                    if ((Form2.text1) == "DRUM BRAKE - FORD R.H")
                    {// To change the score value for both LHS nad RHS 
                        if (i == 0)
                            edgeSearches[cmbxItem].MinScore = 48;
                        if (i == 1)
                            edgeSearches[cmbxItem].MinScore = 42;

                        if (i == 2)
                        {
                            edgeSearches[cmbxItem].MinRotation = -60;
                            edgeSearches[cmbxItem].MaxRotation = 11;
                        }
                        if (i == 3)
                        {//nearly 500components checked rotation value of 2
                            edgeSearches[cmbxItem].MinRotation = -2;
                            edgeSearches[cmbxItem].MaxRotation = 60;
                        }
                        if (i == 4)
                        {
                            edgeSearches[cmbxItem].MinRotation = -180;
                            edgeSearches[cmbxItem].MaxRotation = 180;
                        }
                        if (i == 5)
                        {
                            edgeSearches[cmbxItem].MinRotation = -180;
                            edgeSearches[cmbxItem].MaxRotation = 180;
                            edgeSearches[cmbxItem].MinScore = 45;
                        }
                        if (i == 6)
                        {//nearly 500components checked score value of 45
                            edgeSearches[cmbxItem].MinScore = 32;
                        }
                        if (i == 7)
                            edgeSearches[cmbxItem].MinScore = 10;
                        if (i == 8)
                            edgeSearches[cmbxItem].MinScore = 10;
                        if (i == 10)
                        {//nearly 500components checked score value of 40
                            edgeSearches[cmbxItem].MinScore = 40;
                        }
                        if (i == 11)
                            edgeSearches[cmbxItem].MinScore = 40;
                        if (i == 12)
                        {//nearly 500components checked score value of 35
                            edgeSearches[cmbxItem].MinScore = 30;
                        }
                        if (i == 13)
                        {//nearly 500components checked score value of 30
                            edgeSearches[cmbxItem].MinRotation = -180;
                            edgeSearches[cmbxItem].MaxRotation = 180;
                            edgeSearches[cmbxItem].MinScore = 34;
                        }
                        if (i == 14)
                        {//nearly 500components checked score value of 45
                            edgeSearches[cmbxItem].MinRotation = -180;
                            edgeSearches[cmbxItem].MaxRotation = 180;
                            edgeSearches[cmbxItem].MinScore = 35;
                            
                        }
                        if (i == 16)
                            edgeSearches[cmbxItem].MinScore = 28;
                        if (i ==22)
                            edgeSearches[cmbxItem].MinScore = 55;
                    }
                    else
                    {
                        if (i == 0)
                            edgeSearches[cmbxItem].MinScore = 43;
                        if (i == 1)
                            edgeSearches[cmbxItem].MinScore = 30;
                        if (i == 2)
                        {
                            edgeSearches[cmbxItem].MinRotation = -60;
                            edgeSearches[cmbxItem].MaxRotation = 14;
                        }
                        if (i == 3)
                        {//nearly 500components checked rotation value of 3
                            edgeSearches[cmbxItem].MinRotation = -2;
                            edgeSearches[cmbxItem].MaxRotation = 60;
                        }
                        if (i == 4)
                        {
                            edgeSearches[cmbxItem].MinRotation = -180;
                            edgeSearches[cmbxItem].MaxRotation = 180;
                        }
                        if (i == 5)
                        {
                            
                            edgeSearches[cmbxItem].MinRotation = -180;
                            edgeSearches[cmbxItem].MaxRotation = 180;
                            edgeSearches[cmbxItem].MinScore = 45;

                            
                        }
                        if (i == 6)
                        {//nearly 500components checked score value of 42 & after 38
                            textBox8.Text = (edgeSearches[cmbxItem].MinScore).ToString();
                            edgeSearches[cmbxItem].MinScore = 40;
                        }
                        if (i == 7)
                            edgeSearches[cmbxItem].MinScore = 10;
                        if (i == 8)
                            edgeSearches[cmbxItem].MinScore = 10;
                        if (i == 10)
                            edgeSearches[cmbxItem].MinScore = 40;
                        if (i == 11)
                            edgeSearches[cmbxItem].MinScore = 35;
                        if (i == 12)
                        {//nearly 500components checked score value of 42
                            edgeSearches[cmbxItem].MinScore = 30;
                        }
                        if (i == 13)
                        {//nearly 500components checked score value of 30
                            textBox9.Text = (edgeSearches[cmbxItem].MinScore).ToString();
                            edgeSearches[cmbxItem].MinRotation = -180;
                            edgeSearches[cmbxItem].MaxRotation = 180;
                            edgeSearches[cmbxItem].MinScore = 34;
                        }
                        if (i == 14)
                        {//nearly 500components checked score value of 45
                            textBox10.Text = (edgeSearches[cmbxItem].MinScore).ToString();
                            edgeSearches[cmbxItem].MinRotation = -180;
                            edgeSearches[cmbxItem].MaxRotation = 180;
                            edgeSearches[cmbxItem].MinScore = 35;
                        }
                        if (i == 16)
                            edgeSearches[cmbxItem].MinScore = 5;
                    }
                    // Execute two times
                    // The first execution may be slowed-down by preprocessing
                    m_Timer.Reset();

                    edgeSearches[cmbxItem].Execute(testimg);
                    m_ExecuteTime1 = m_Timer.GetTime(true) * 1000F;
                    //tlStrpPrgsBar1.Value = 60;
                    //searches[i].Execute(imgIn);
                    //m_ExecuteTime2 = m_Timer.GetTime(false) * 1000F;

                    // Put back default searches[i] parameters
                    edgeSearches[cmbxItem].MaxMatches = maxMatches;
                    edgeSearches[cmbxItem].MinRotation = minRotation;
                    edgeSearches[cmbxItem].MaxRotation = maxRotation;
                    pass_fail[i] = edgeSearches[cmbxItem].GetNumMatches() >= 1;
                }
                else if (enableSearchArea[i])
                {
                    // Remember default search parameters
                    int maxMatches = areaSearches[cmbxItem].MaxMatches;
                    tlStrpPrgsBar1.Value = 40;
                    // Set search parameters
                    areaSearches[cmbxItem].MinScore = (int)numericUpDown1.Value;
                    if (i == 1)
                    {
                        areaSearches[cmbxItem].MinScore = 50;
                        //edgeSearches[cmbxItem].MinRotation = -45;
                        //edgeSearches[cmbxItem].MaxRotation = 45;

                    }
                    areaSearches[cmbxItem].MaxMatches = 1;
                    // Execute two times
                    // The first execution may be slowed-down by preprocessing
                    m_Timer.Reset();
                    areaSearches[cmbxItem].Execute(testimg);
                    m_ExecuteTime1 = m_Timer.GetTime(true) * 1000F;

                    // Put back default searches[i] parameters
                    areaSearches[cmbxItem].MaxMatches = maxMatches;
                    //areaSearches[cmbxItem].MinRotation = minRotation;
                    //areaSearches[cmbxItem].MaxRotation = maxRotation;
                    pass_fail[i] = areaSearches[cmbxItem].GetNumMatches() >= 1;
                }
                if (enableHoughCircle[i])
                {
                    // Set parameters to default values
                    HoughCircles[cmbxItem].ResetParameters();

                    // Set parameters
                    HoughCircles[cmbxItem].SegmentationType = CProHough.SegmentationTypeEnum.SegmentationEdge;
                    HoughCircles[cmbxItem].EdgeStrengthMode = CProHough.EdgeStrengthModeEnum.EdgeStrengthLow;
                    HoughCircles[cmbxItem].EdgeStrength = 15;
                    // for pinposition finding
                    if ((i == 30) || (i == 31))
                    {
                        HoughCircles[cmbxItem].MaxCircles = 1;
                        HoughCircles[cmbxItem].CenterVicinity = 10.0F;
                        CProRect circle_roi = rectRois[cmbxItem];
                        //textBox2.Text=
                        HoughCircles[cmbxItem].StartRadius = 12.0F;
                        HoughCircles[cmbxItem].EndRadius = 18.0F;
                        HoughCircles[cmbxItem].MinPtsPerCircle = 15;

                        //CProPoint s=HoughCircles[cmbxItem].
                        HoughCircles[cmbxItem].Execute(testimg);
                        centerPoint = HoughCircles[cmbxItem].GetResultCenterPoint(0);
                        centerPoint.x = centerPoint.x + circle_roi.x;
                        centerPoint.y = centerPoint.y + circle_roi.y;
                        pincenter = centerPoint.y;
                        float r = HoughCircles[cmbxItem].GetResultRadius(0);
                        if ((i == 30))
                        {
                            //CProFPoint centerPoint = HoughCircles[cmbxItem].GetResultCenterPoint(i);
                            float outang; float outdist; float ang = (float)((l1.GetResultAngle(0) * 1.5708) / 90);
                            perRline.LinePerpendicular(ang, centerPoint, out outang, out outdist);
                           // g.DrawLine(pen, centerPoint.x, centerPoint.y, outdist, outang);
                            CProFPoint inter;
                            //intersectline.Line2LineIntersection(ang, k, 0, (float)getsubrpix, out inter);
                            // intersectline.Line2LineIntersection(ang, centerPoint.x, l1.GetResultAngle(0), (float)getsubrpix, out inter);
                            // g.DrawLine(Pens.Red, centerPoint.x, centerPoint.y, inter.x, inter.y);
                            float sectanglee; float outorgy;
                            segRline.Segment2Line(l1.GetResultIntersectPoint1(0), l1.GetResultIntersectPoint2(0), out sectanglee, out outorgy);
                            //intersectline.Line2LineIntersection(l1.GetResultAngle(0), orgy, 0, k, out inter);
                            intersectRline.Line2LineIntersection(ang, outorgy, outang, outdist, out inter);
                            float dist;
                            // pintersectline.
                            //distance.Point2PointDist(centerPoint, inter, out dist);
                            textBox4.Text = ((centerPoint.y) - (inter.y)).ToString();
                            pindist = Convert.ToInt32((centerPoint.y) - (inter.y));
                            //g.DrawLine(Pens.Red, centerPoint.x, centerPoint.y, inter.x, inter.y);

                        }
                        else if(i==31)
                        {
                            //CProFPoint centerPoint = HoughCircles[cmbxItem].GetResultCenterPoint(i);
                            float outang; float outdist; float ang = (float)((shdl1.GetResultAngle(0) * 1.5708) / 90);
                            perLline.LinePerpendicular(ang, centerPoint, out outang, out outdist);
                            // g.DrawLine(pen, centerPoint.x, centerPoint.y, outdist, outang);
                            CProFPoint inter;
                            //intersectline.Line2LineIntersection(ang, k, 0, (float)getsubrpix, out inter);
                            // intersectline.Line2LineIntersection(ang, centerPoint.x, l1.GetResultAngle(0), (float)getsubrpix, out inter);
                            // g.DrawLine(Pens.Red, centerPoint.x, centerPoint.y, inter.x, inter.y);
                            float sectanglee; float outorgy;
                            segLline.Segment2Line(shdl1.GetResultIntersectPoint1(0), shdl1.GetResultIntersectPoint2(0), out sectanglee, out outorgy);
                            //intersectline.Line2LineIntersection(l1.GetResultAngle(0), orgy, 0, k, out inter);
                            intersectLline.Line2LineIntersection(ang, outorgy, outang, outdist, out inter);
                            float dist;
                            // pintersectline.
                            //distance.Point2PointDist(centerPoint, inter, out dist);
                            textBox6.Text = ((centerPoint.y) - (inter.y)).ToString();
                            pindist = Convert.ToInt32((centerPoint.y) - (inter.y));
                        }

                        pass_fail[i] = (pindist> 80)&&(pindist<150);

                    }
                    else
                    {
                        HoughCircles[cmbxItem].MaxCircles = 2;
                        HoughCircles[cmbxItem].CenterVicinity = 50.0F;
                        HoughCircles[cmbxItem].StartRadius = 20.0F;
                        HoughCircles[cmbxItem].EndRadius = 2500.0F;
                        HoughCircles[cmbxItem].MinPtsPerCircle = 50;
                        HoughCircles[cmbxItem].Execute(testimg);
                        float r = HoughCircles[cmbxItem].GetResultRadius(0);
                        pass_fail[i] = (r > 35) || (r < 55);
                    }

                    // Process the transformation




                }
                if (cmbxItem == "AssemblyDia")
                {
                    //preproc.Equalize(testimg, testimg, 0, 41, 0, 255);
                    CProRect temprect = rectRois[cmbxItem];
                    CProPoint cent = new CProPoint((int)(temprect.x + temprect.width / 2), (int)(temprect.y + temprect.height / 2));
                    CProPoint crl1 = new CProPoint(cent.x, (int)(temprect.y + Math.Min(temprect.height, temprect.width) * 0.992));
                    CProPoint crl2 = new CProPoint(cent.x, (int)(temprect.y + Math.Min(temprect.height, temprect.width) * 0.92));
                    spoke1.Center = cent;
                    spoke1.CirclePoint1 = crl1;
                    spoke1.CirclePoint2 = crl2;
                    spoke1.NumElement = 150;
                    spoke1.EdgeType = CProMeasMarker.EdgeTypeEnum.EdgeFalling;
                    spoke1.SearchDirection = CProMeasMarker.SearchDirectionEnum.SearchForward;
                    spoke1.LevelType = CProMeasMarker.LevelTypeEnum.LevelAbsolute;
                    spoke1.LevelThresh = 20;
                    spoke1.MaxEdges = 1;
                    spoke1.Execute(testimg);
                    pass_fail[i] = (spoke1.GetFittedRadius() > 830) && (spoke1.GetFittedRadius() < 850);

                }
                if (cmbxItem == "ShoeDia")
                    pass_fail[i] = true;

                if (cmbxItem == "shdpin")
                {
                    pass_fail[i] = true;
                }
                if (cmbxItem == "SHDLPIN")
                {
                    pass_fail[i] = true;
                }
                if ((cmbxItem == "SHDRTop"))
                {

                    //preproc.Equalize(testimg, testimg, 0, 101, 0, 255);
                    //preproc.Thresh(testimg, testimg, 82, 255, false,CProBasic.ThreshLevelType.ThreshLevelDouble,CProBasic.ThreshResultType.ThreshResultBinary);
                    //Mat temp = new Mat(testimg.Height, testimg.Width, MatType.CV_8UC1, testimg.GetData());
                    //Mat temp3 = new Mat(temp, new Rect(rectRois[cmbxItem].x, rectRois[cmbxItem].y, rectRois[cmbxItem].width, rectRois[cmbxItem].height));
                    //Mat dest = new Mat();
                    //Cv2.Reduce(temp3.Clone(), dest, ReduceDimension.Column, ReduceTypes.Sum, MatType.CV_32SC1);
                    //Mat temp2 = new Mat();
                    //Cv2.Repeat(dest, 1, temp3.Width, temp2);
                    //Cv2.Normalize(temp2, temp2, 0, 255, NormTypes.MinMax);
                    //temp2.ConvertTo(temp3, MatType.CV_8UC1);
                    //Cv2.ImShow("l", temp3);
                    //using (new Window("in", WindowMode.Normal, temp2)) ;
                    //using (new Window("proj", WindowMode.Normal, temp)) ;
                    //Cv2.ImWrite("testline.bmp", temp);
                    //CProImage img = new CProImage(temp.Width, temp.Height, CProData.FormatEnum.FormatUByte, rectRois[cmbxItem], temp.Data, true);

                    //l1.MinPtsPerLine = 40;
                    //l1.MaxLines = 1;
                    //l1.SegmentationType = CProHough.SegmentationTypeEnum.SegmentationEdge;
                    //l1.AngleResolution = 1;
                    //l1.EdgeStrengthMode = CProHough.EdgeStrengthModeEnum.EdgeStrengthManual;
                    //l1.EdgeStrength = 15;
                    //l1.SmoothFilterSize = CProHough.SmoothFilterSizeEnum.Smooth5x5;
                    //l1.AngleVicinity = 1;
                    //l1.RadialDistanceResolution = 1; l1.RadialDistanceVicinity = 1;

                    //l1.StartAngle = 0;
                    //l1.EndAngle = 180;
                    //l1.Execute(testimg);
                    //CProImage inputs = new CProImage(imgIn.Width, imgIn.Height, CProData.FormatEnum.FormatUByte, rectRois["SHDR"], imgIn.GetData(), true);
                    //shdmeasure.Set(inputs.Width, inputs.Height, CProData.FormatEnum.FormatUByte, rectRois["SHDR"], inputs.GetData(), true);
                    //CProImage temp = new CProImage();
                    //shdmeasure.Set(imgIn.Width, imgIn.Height, CProData.FormatEnum.FormatUByte, rectRois["SHDR"], imgIn.GetData(), true);
                    //preproc.Rotate(inputs, shdmeasure, l1.GetResultAngle(0) - 90, CProBasic.RotateSizeOp.RotateSizeClip);

                    //Mat rotate = new Mat(shdmeasure.Height, shdmeasure.Width, MatType.CV_8UC1, shdmeasure.GetData());

                    //temp = new Mat(shdmeasure.Height, shdmeasure.Width, MatType.CV_8UC1, shdmeasure.GetData());
                    //Cv2.ImWrite("d.bmp", temp);

                    //textBox3.Text = l1.GetResultAngle(0).ToString();
                    //shdmeasure.Set(testimg.Width, testimg.Height,CProData.FormatEnum.FormatUByte,rectRois["SHDR"],imgIn.GetData(), true);
                    //preproc.Rotate(shdmeasure, shdmeasure, l1.GetResultAngle(1));
                    //CProRect roirect = rectRois[cmbxItem];
                    //int stx = roirect.x - rectRois["SHDR"].x;
                    //int sty = roirect.y - rectRois["SHDR"].y;
                    //CProRect topr = new CProRect(stx, sty, roirect.width, roirect.height);
                    //stx = rectRois["left"].x - rectRois["SHDR"].x;
                    //sty = rectRois["left"].y - rectRois["SHDR"].y;
                    //CProRect leftr = new CProRect(stx, sty, rectRois["left"].width, rectRois["left"].height);
                    //stx = rectRois["right"].x - rectRois["SHDR"].x;
                    //sty = rectRois["right"].y - rectRois["SHDR"].y;
                    //CProRect rightr = new CProRect(stx, sty, rectRois["right"].width, rectRois["right"].height);
                    //stx = rectRois["bottom"].x - rectRois["SHDR"].x;
                    //sty = rectRois["bottom"].y - rectRois["SHDR"].y;
                    //CProRect bottomr = new CProRect(stx, sty, rectRois["bottom"].width, rectRois["bottom"].height);
                    //preproc.Rotate()
                    // preproc.Equalize(testimg, testimg, 0, 101, 0, 255);
                    // preproc.Thresh(testimg, testimg, 32, 133, true, CProBasic.ThreshLevelType.ThreshLevelDouble, CProBasic.ThreshResultType.ThreshResultBinary);
                    //rake1.Rect = temprect;
                    //rake1.NumElement = 50;
                    //rake1.EdgeType = CProMeasMarker.EdgeTypeEnum.EdgeFalling;
                    //rake1.SearchDirection = CProMeasMarker.SearchDirectionEnum.SearchForward;
                    //rake1.LevelType = CProMeasMarker.LevelTypeEnum.LevelAbsolute;
                    //rake1.LevelThresh = 20;
                    //rake1.MaxEdges = 1;
                    //rake1.Smoothing = 15;
                    //rake1.Execute(testimg);
                    pass_fail[i] = true;
                }
                if (cmbxItem == "SHDRBottom")
                {
                    //    testimg = opencvfnprojection(testimg, rectRois[cmbxItem], false);
                    //    Mat test = new Mat(testimg.Height, testimg.Width, MatType.CV_8UC1, testimg.GetData());
                    //    Cv2.ImWrite("test.bmp", test);
                    //    //Mat temp = new Mat(testimg.Height, testimg.Width, MatType.CV_8UC1, testimg.GetData());
                    //    //CProImage img;
                    //    //img = new CProImage(temp.Width, temp.Height, CProData.FormatEnum.FormatUByte, rectRois[cmbxItem], temp.Data, true);

                    //    l2.MinPtsPerLine = 8;
                    //    l2.MaxLines = 1;
                    //    l2.SegmentationType = CProHough.SegmentationTypeEnum.SegmentationEdge;
                    //    l2.AngleResolution = 1;
                    //    l2.EdgeStrengthMode = CProHough.EdgeStrengthModeEnum.EdgeStrengthManual;
                    //    l2.EdgeStrength = 5;
                    //    l2.SmoothFilterSize = CProHough.SmoothFilterSizeEnum.Smooth5x5;
                    //    l2.AngleVicinity = 1;
                    //    l2.RadialDistanceResolution = 1; l2.RadialDistanceVicinity = 1;

                    //    l2.StartAngle = 0;
                    //    l2.EndAngle = 180;
                    //    l2.Execute(testimg);
                    //    Mat test1 = new Mat(testimg.Height, testimg.Width, MatType.CV_8UC1, testimg.GetData());
                    //    Cv2.ImWrite("test.bmp", test1);
                    //preproc.Equalize(testimg, testimg, 0, 101, 0, 255);
                    //preproc.Thresh(testimg, testimg, 82, 255, false,CProBasic.ThreshLevelType.ThreshLevelDouble,CProBasic.ThreshResultType.ThreshResultBinary);
                    //Mat temp = new Mat(testimg.Height, testimg.Width, MatType.CV_8UC1, testimg.GetData());
                    //    Mat temp3 = new Mat(temp, new Rect(rectRois[cmbxItem].x, rectRois[cmbxItem].y, rectRois[cmbxItem].width, rectRois[cmbxItem].height));
                    //    Mat dest = new Mat();
                    //    Cv2.Reduce(temp3.Clone(), dest, ReduceDimension.Column, ReduceTypes.Sum, MatType.CV_32SC1);
                    //    Mat temp2 = new Mat();
                    //    Cv2.Repeat(dest, 1, temp3.Width, temp2);
                    //    Cv2.Normalize(temp2, temp2, 0, 255, NormTypes.MinMax);
                    //    temp2.ConvertTo(temp3, MatType.CV_8UC1);
                    //    //using (new Window("in", WindowMode.Normal, temp2)) ;
                    //     //using (new Window("proj", WindowMode.Normal, temp)) ;
                    //    //Cv2.ImWrite("testline2.bmp", temp);
                    //    CProImage img = new CProImage(temp.Width, temp.Height, CProData.FormatEnum.FormatUByte, rectRois[cmbxItem], temp.Data, true);
                    //    preproc.Thresh(img, img, 110, 255, false, CProBasic.ThreshLevelType.ThreshLevelSingle, CProBasic.ThreshResultType.ThreshResultBinary);
                    //    l2.MinPtsPerLine = 40;
                    //    l2.MaxLines = 1;
                    //l2.SegmentationType = CProHough.SegmentationTypeEnum.SegmentationEdge;
                    //l2.AngleResolution = 1;
                    //l2.EdgeStrengthMode = CProHough.EdgeStrengthModeEnum.EdgeStrengthManual;
                    //    l2.EdgeStrength = 20;
                    //    l2.SmoothFilterSize = CProHough.SmoothFilterSizeEnum.Smooth5x5;
                    //l2.AngleVicinity = 1;
                    //l2.RadialDistanceResolution = 1; l2.RadialDistanceVicinity = 1;

                    //l2.StartAngle = 0;
                    //l2.EndAngle = 180;
                    //l2.Execute(img);
                        pass_fail[i] = true;
                        // textBox2.Text = l2.ToString();
                    }
                    if (cmbxItem == "SHDRLeft")
                {

                    //preproc.Equalize(testimg, testimg, 0, 101, 0, 255);
                    //preproc.Thresh(testimg, testimg, 82, 255, false,CProBasic.ThreshLevelType.ThreshLevelDouble,CProBasic.ThreshResultType.ThreshResultBinary);
                    //Mat temp = new Mat(testimg.Height, testimg.Width, MatType.CV_8UC1, testimg.GetData());
                    //Mat temp3 = new Mat(temp, new Rect(rectRois[cmbxItem].x, rectRois[cmbxItem].y, rectRois[cmbxItem].width, rectRois[cmbxItem].height));
                    //Mat dest = new Mat();
                    //Cv2.Reduce(temp3.Clone(), dest, ReduceDimension.Row, ReduceTypes.Sum, MatType.CV_32SC1);
                    //Mat temp2 = new Mat();
                    //Cv2.Repeat(dest, temp3.Height, 1, temp2);
                    //Cv2.Normalize(temp2, temp2, 0, 255, NormTypes.MinMax);
                    //temp2.ConvertTo(temp3, MatType.CV_8UC1);
                    ////using (new Window("in", WindowMode.Normal, temp2)) ;
                    //// using (new Window("proj", WindowMode.Normal, temp)) ;
                    ////Cv2.ImWrite("testline.bmp", temp);
                    //CProImage img = new CProImage(temp.Width, temp.Height, CProData.FormatEnum.FormatUByte, rectRois[cmbxItem], temp.Data, true);
                    //l3.MinPtsPerLine = 40;
                    //l3.MaxLines = 1;
                    //l3.SegmentationType = CProHough.SegmentationTypeEnum.SegmentationEdge;
                    //l3.AngleResolution = 1;
                    //l3.EdgeStrengthMode = CProHough.EdgeStrengthModeEnum.EdgeStrengthManual;
                    //l3.EdgeStrength = 30;
                    //l3.SmoothFilterSize = CProHough.SmoothFilterSizeEnum.Smooth5x5;
                    //l3.AngleVicinity = 1;
                    //l3.RadialDistanceResolution = 1; l3.RadialDistanceVicinity = 1;

                    //l3.StartAngle = 0;
                    //l3.EndAngle = 180;
                    //l3.Execute(img);
                    pass_fail[i] = true;
                }
                if (cmbxItem == "SHDRRight")
                {

                    //preproc.Equalize(testimg, testimg, 0, 101, 0, 255);
                    //preproc.Thresh(testimg, testimg, 82, 255, false,CProBasic.ThreshLevelType.ThreshLevelDouble,CProBasic.ThreshResultType.ThreshResultBinary);
                    //Mat temp = new Mat(testimg.Height, testimg.Width, MatType.CV_8UC1, testimg.GetData());
                    //Mat temp3 = new Mat(temp, new Rect(rectRois[cmbxItem].x, rectRois[cmbxItem].y, rectRois[cmbxItem].width, rectRois[cmbxItem].height));
                    //Mat dest = new Mat();
                    //Cv2.Reduce(temp3.Clone(), dest, ReduceDimension.Row, ReduceTypes.Sum, MatType.CV_32SC1);
                    //Mat temp2 = new Mat();
                    //Cv2.Repeat(dest, temp3.Height, 1, temp2);
                    //Cv2.Normalize(temp2, temp2, 0, 255, NormTypes.MinMax);
                    //temp2.ConvertTo(temp3, MatType.CV_8UC1);

                    ////using (new Window("in", WindowMode.Normal, temp2)) ;
                    //// using (new Window("proj", WindowMode.Normal, temp)) ;
                    ////Cv2.ImWrite("testline.bmp", temp);
                    //CProImage img = new CProImage(temp.Width, temp.Height, CProData.FormatEnum.FormatUByte, rectRois[cmbxItem], temp.Data, true);
                    //l4.MinPtsPerLine = 40;
                    //l4.MaxLines = 1;
                    //l4.SegmentationType = CProHough.SegmentationTypeEnum.SegmentationEdge;
                    //l4.AngleResolution = 1;
                    //l4.EdgeStrengthMode = CProHough.EdgeStrengthModeEnum.EdgeStrengthManual;
                    //l4.EdgeStrength = 20;
                    //l4.SmoothFilterSize = CProHough.SmoothFilterSizeEnum.Smooth5x5;
                    //l4.AngleVicinity = 1;
                    //l4.RadialDistanceResolution = 1; l4.RadialDistanceVicinity = 1;

                    //l4.StartAngle = 0;
                    //l4.EndAngle = 180;
                    //l4.Execute(img);
                    pass_fail[i] = true;
                    //if (l1.GetResultNumLines() == 1 && l2.GetResultNumLines() == 1 && l3.GetResultNumLines() == 1 && l4.GetResultNumLines() == 1)
                    //{
                    //    shdrhg.Text = Math.Sqrt(Math.Pow((l1.GetResultIntersectPoint1(0).x - l2.GetResultIntersectPoint1(0).x), 2) + Math.Pow((l1.GetResultIntersectPoint1(0).y - l2.GetResultIntersectPoint1(0).y), 2)).ToString();
                    //    shdrwd.Text = Math.Sqrt(Math.Pow((l3.GetResultIntersectPoint1(0).x - l4.GetResultIntersectPoint1(0).x), 2) + Math.Pow((l3.GetResultIntersectPoint1(0).y - l4.GetResultIntersectPoint1(0).y), 2)).ToString();
                    //    rh = Math.Sqrt(Math.Pow((l1.GetResultIntersectPoint1(0).x - l2.GetResultIntersectPoint1(0).x), 2) + Math.Pow((l1.GetResultIntersectPoint1(0).y - l2.GetResultIntersectPoint1(0).y), 2));
                    //    rw = Math.Sqrt(Math.Pow((l3.GetResultIntersectPoint1(0).x - l4.GetResultIntersectPoint1(0).x), 2) + Math.Pow((l3.GetResultIntersectPoint1(0).y - l4.GetResultIntersectPoint1(0).y), 2));
                    //    pass_fail[i] = (rh > 10) && (rw > 1);

                    //}
                }

                if (cmbxItem == "Clr")
                {
                    Mat temp = new Mat(testimg.Height, testimg.Width, MatType.CV_8UC1, testimg.GetData());
                    Mat temp3 = new Mat(temp, new Rect(rectRois[cmbxItem].x, rectRois[cmbxItem].y, rectRois[cmbxItem].width, rectRois[cmbxItem].height));
                    //Cv2.ImShow("in", temp3);
                    int countpixel = 0;
                    if ((Form2.text2 == "LH MODEL1")|| (Form2.text2 == "RH MODEL1"))
                    {
                        //preproc.CountPix(testimg, out int b, 100, 200);
                        //textBox1.Text = b.ToString();
                        
                        //if (Cv2.CountNonZero(temp3)> 100)
                        //{
                        //    textBox1.Text = Cv2.CountNonZero(temp3).ToString();
                        //    pass_fail[i] = true; ;
                        //}
                        for (int x = 0; x < temp3.Rows; x++)
                        {
                            for (int y = 0; y < temp3.Cols; y++)
                            {
                                if(temp3.At<char>(x,y)>=200)
                                {
                                    countpixel++;
                                }
                            }
                        }
                        if(countpixel>900)
                        {
                            textBox1.Text = countpixel.ToString();
                            pass_fail[i] = true;
                        }
                        else
                        {
                            pass_fail[i] = false;
                        }
                    }
                    else if ((Form2.text2 == "LH MODEL2")|| (Form2.text2 == "RH MODEL2"))
                    {
                        //preproc.CountPix(testimg, out int b, 100, 200);
                        //textBox1.Text = b.ToString();
                        //pass_fail[i] = b < 900;
                        for (int x = 0; x < temp3.Rows; x++)
                        {
                            for (int y = 0; y < temp3.Cols; y++)
                            {
                                if (temp3.At<char>(x, y) >= 200)
                                {
                                    countpixel++;
                                }
                            }
                        }
                        if (countpixel < 1400)
                        {
                            textBox1.Text = countpixel.ToString();
                            pass_fail[i] = true;
                        }
                        else
                        {
                           
                            
                                pass_fail[i] = false;
                            
                        }
                    }
                }
                if (cmbxItem == "clrright")
                {
                    Mat temp = new Mat(testimg.Height, testimg.Width, MatType.CV_8UC1, testimg.GetData());
                    Mat temp3 = new Mat(temp, new Rect(rectRois[cmbxItem].x, rectRois[cmbxItem].y, rectRois[cmbxItem].width, rectRois[cmbxItem].height));
                    //Cv2.ImShow("in", temp3);
                    int countpixel2 = 0;
                    if ((Form2.text2 == "LH MODEL1")|| (Form2.text2 == "RH MODEL1"))
                    {
                        //preproc.CountPix(testimg, out int b, 100, 200);
                        //textBox2.Text = b.ToString();

                        //pass_fail[i] = b > 1000;
                        for (int x = 0; x < temp3.Rows; x++)
                        {
                            for (int y = 0; y < temp3.Cols; y++)
                            {
                                if (temp3.At<char>(x, y) >= 200)
                                {
                                    countpixel2++;
                                }
                            }
                        }
                        if (countpixel2 > 900)
                        {
                           
                            textBox2.Text = countpixel2.ToString();
                            pass_fail[i] = true;
                            pass_fail[29] = true;
                        }
                        else
                        {

                            if (pass_fail[29] == true)
                            {
                                pass_fail[i] = true;
                            }
                            else
                            {
                                pass_fail[i] = false;
                            }
                        }
                    }
                    else if ((Form2.text2 == "LH MODEL2") || (Form2.text2 == "RH MODEL2"))
                    {
                        //preproc.CountPix(testimg, out int b, 100, 200);
                        //textBox2.Text = b.ToString();

                        //pass_fail[i] = b<750;
                        for (int x = 0; x < temp3.Rows; x++)
                        {
                            for (int y = 0; y < temp3.Cols; y++)
                            {
                                if (temp3.At<char>(x, y) >= 200)
                                {
                                    countpixel2++;
                                }
                            }
                        }
                        if (countpixel2 < 1400)
                        {
                            textBox2.Text = countpixel2.ToString();
                            pass_fail[i] = true;
                            pass_fail[29] = true;
                        }
                        else
                        {
                            if (pass_fail[29] == true)
                            {
                                pass_fail[i] = true;
                            }
                            else
                            {
                                pass_fail[i] = false;
                            }
                        }
                    }
                }

                if ((cmbxItem == "SHDLTop"))
                {

                    //preproc.Equalize(testimg, testimg, 0, 101, 0, 255);
                    //preproc.Thresh(testimg, testimg, 82, 255, false,CProBasic.ThreshLevelType.ThreshLevelDouble,CProBasic.ThreshResultType.ThreshResultBinary);
                    //Mat temp = new Mat(testimg.Height, testimg.Width, MatType.CV_8UC1, testimg.GetData());
                    //Mat temp3 = new Mat(temp, new Rect(rectRois[cmbxItem].x, rectRois[cmbxItem].y, rectRois[cmbxItem].width, rectRois[cmbxItem].height));
                    ////Mat dest = new Mat();
                    ////Cv2.Reduce(temp3.Clone(), dest, ReduceDimension.Column, ReduceTypes.Sum, MatType.CV_32SC1);
                    ////Mat temp2 = new Mat();
                    ////Cv2.Repeat(dest, 1, temp3.Width, temp2);
                    ////Cv2.Normalize(temp2, temp2, 0, 255, NormTypes.MinMax);
                    ////temp2.ConvertTo(temp3, MatType.CV_8UC1);
                    ////Cv2.ImShow("l", temp);
                    //// Cv2.ImShow("l", temp3);
                    ////using (new Window("in", WindowMode.Normal, temp2)) ;
                    //// using (new Window("proj", WindowMode.Normal, temp)) ;
                    ////Cv2.ImWrite("testline.bmp", temp);
                    //CProImage img = new CProImage(temp.Width, temp.Height, CProData.FormatEnum.FormatUByte, rectRois[cmbxItem], temp.Data, true);
                    //shdl1.SmoothFilterSize = CProHough.SmoothFilterSizeEnum.Smooth5x5;
                    //shdl1.SegmentationType = CProHough.SegmentationTypeEnum.SegmentationEdge;
                    //shdl1.AngleResolution = 1;
                    //shdl1.AngleVicinity = 1;
                    //shdl1.RadialDistanceResolution = 1; shdl1.RadialDistanceVicinity = 1;

                    //shdl1.EdgeStrengthMode = CProHough.EdgeStrengthModeEnum.EdgeStrengthManual;
                    //shdl1.EdgeStrength = 28;
                    //shdl1.MaxLines = 1;
                    //shdl1.MinPtsPerLine = 30;


                    //shdl1.StartAngle = 0;
                    //shdl1.EndAngle = 180;

                    //shdl1.Execute(img);
                    //CProImage inputs = new CProImage(imgIn.Width, imgIn.Height, CProData.FormatEnum.FormatUByte, rectRois["SHDL"], imgIn.GetData(), true);

                    //shdmeasure.Set(imgIn.Width, imgIn.Height, CProData.FormatEnum.FormatUByte, rectRois["SHDL"], imgIn.GetData(), true);
                    //preproc.Rotate(inputs, shdmeasure, shdl1.GetResultAngle(0) - 90, CProBasic.RotateSizeOp.RotateSizeClip);

                    //Mat rotate = new Mat(shdmeasure.Height, shdmeasure.Width, MatType.CV_8UC1, shdmeasure.GetData());

                    //temp = new Mat(shdmeasure.Height, shdmeasure.Width, MatType.CV_8UC1, shdmeasure.GetData());

                    //getsublpix = ((shdl1.GetResultIntersectPoint2(0).y) - (shdl1.GetResultIntersectPoint1(0).y)) / 2;
                    //getlpix = (l1.GetResultIntersectPoint1(0).y) + getsublpix;
                    //shdmeasure.Set(testimg.Width, testimg.Height,CProData.FormatEnum.FormatUByte,rectRois["SHDR"],imgIn.GetData(), true);
                    //preproc.Rotate(shdmeasure, shdmeasure, l1.GetResultAngle(1));
                    //CProRect roirect = rectRois[cmbxItem];
                    //int stx = roirect.x - rectRois["SHDR"].x;
                    //int sty = roirect.y - rectRois["SHDR"].y;
                    //CProRect topr = new CProRect(stx, sty, roirect.width, roirect.height);
                    //stx = rectRois["left"].x - rectRois["SHDR"].x;
                    //sty = rectRois["left"].y - rectRois["SHDR"].y;
                    //CProRect leftr = new CProRect(stx, sty, rectRois["left"].width, rectRois["left"].height);
                    //stx = rectRois["right"].x - rectRois["SHDR"].x;
                    //sty = rectRois["right"].y - rectRois["SHDR"].y;
                    //CProRect rightr = new CProRect(stx, sty, rectRois["right"].width, rectRois["right"].height);
                    //stx = rectRois["bottom"].x - rectRois["SHDR"].x;
                    //sty = rectRois["bottom"].y - rectRois["SHDR"].y;
                    //CProRect bottomr = new CProRect(stx, sty, rectRois["bottom"].width, rectRois["bottom"].height);
                    //preproc.Rotate()
                    // preproc.Equalize(testimg, testimg, 0, 101, 0, 255);
                    // preproc.Thresh(testimg, testimg, 32, 133, true, CProBasic.ThreshLevelType.ThreshLevelDouble, CProBasic.ThreshResultType.ThreshResultBinary);
                    //rake1.Rect = temprect;
                    //rake1.NumElement = 50;
                    //rake1.EdgeType = CProMeasMarker.EdgeTypeEnum.EdgeFalling;
                    //rake1.SearchDirection = CProMeasMarker.SearchDirectionEnum.SearchForward;
                    //rake1.LevelType = CProMeasMarker.LevelTypeEnum.LevelAbsolute;
                    //rake1.LevelThresh = 20;
                    //rake1.MaxEdges = 1;
                    //rake1.Smoothing = 15;
                    //rake1.Execute(testimg);
                    pass_fail[i] = true;
                }
                if (cmbxItem == "SHDLBottom")
                {

                    //preproc.Equalize(testimg, testimg, 0, 101, 0, 255);
                    //preproc.Thresh(testimg, testimg, 82, 255, false,CProBasic.ThreshLevelType.ThreshLevelDouble,CProBasic.ThreshResultType.ThreshResultBinary);
                    //Mat temp = new Mat(testimg.Height, testimg.Width, MatType.CV_8UC1, testimg.GetData());
                    //Mat temp3 = new Mat(temp, new Rect(rectRois[cmbxItem].x, rectRois[cmbxItem].y, rectRois[cmbxItem].width, rectRois[cmbxItem].height));
                    //Mat dest = new Mat();
                    //Cv2.Reduce(temp3.Clone(), dest, ReduceDimension.Column, ReduceTypes.Sum, MatType.CV_32SC1);
                    //Mat temp2 = new Mat();
                    //Cv2.Repeat(dest, 1, temp3.Width, temp2);
                    //Cv2.Normalize(temp2, temp2, 0, 255, NormTypes.MinMax);
                    //temp2.ConvertTo(temp3, MatType.CV_8UC1);
                    ////using (new Window("in", WindowMode.Normal, temp2)) ;
                    ////using (new Window("proj", WindowMode.Normal, temp)) ;
                    ////Cv2.ImWrite("testline2.bmp", temp);
                    //CProImage img = new CProImage(temp.Width, temp.Height, CProData.FormatEnum.FormatUByte, rectRois[cmbxItem], temp.Data, true);
                    //preproc.Thresh(img, img, 110, 255, false, CProBasic.ThreshLevelType.ThreshLevelSingle, CProBasic.ThreshResultType.ThreshResultBinary);
                    //shdl2.MinPtsPerLine = 40;
                    //shdl2.MaxLines = 1;
                    //shdl2.SegmentationType = CProHough.SegmentationTypeEnum.SegmentationEdge;
                    //shdl2.AngleResolution = 1;

                    //shdl2.EdgeStrengthMode = CProHough.EdgeStrengthModeEnum.EdgeStrengthManual;
                    //shdl2.EdgeStrength = 10;
                    //shdl2.SmoothFilterSize = CProHough.SmoothFilterSizeEnum.Smooth5x5;
                    //shdl2.AngleVicinity = 1;
                    //shdl2.RadialDistanceResolution = 1; shdl2.RadialDistanceVicinity = 1;
                    //shdl2.StartAngle = 0;
                    //shdl2.EndAngle = 180;
                    //shdl2.Execute(img);
                    pass_fail[i] = true;
                }
                if (cmbxItem == "SHDLLeft")
                {

                    //preproc.Equalize(testimg, testimg, 0, 101, 0, 255);
                    //preproc.Thresh(testimg, testimg, 82, 255, false,CProBasic.ThreshLevelType.ThreshLevelDouble,CProBasic.ThreshResultType.ThreshResultBinary);
                    //Mat temp = new Mat(testimg.Height, testimg.Width, MatType.CV_8UC1, testimg.GetData());
                    //Mat temp3 = new Mat(temp, new Rect(rectRois[cmbxItem].x, rectRois[cmbxItem].y, rectRois[cmbxItem].width, rectRois[cmbxItem].height));
                    //Mat dest = new Mat();
                    //Cv2.Reduce(temp3.Clone(), dest, ReduceDimension.Row, ReduceTypes.Sum, MatType.CV_32SC1);
                    //Mat temp2 = new Mat();
                    //Cv2.Repeat(dest, temp3.Height, 1, temp2);
                    //Cv2.Normalize(temp2, temp2, 0, 255, NormTypes.MinMax);
                    //temp2.ConvertTo(temp3, MatType.CV_8UC1);
                    ////using (new Window("in", WindowMode.Normal, temp2)) ;
                    //// using (new Window("proj", WindowMode.Normal, temp)) ;
                    ////Cv2.ImWrite("testline.bmp", temp);
                    //CProImage img = new CProImage(temp.Width, temp.Height, CProData.FormatEnum.FormatUByte, rectRois[cmbxItem], temp.Data, true);
                    //shdl3.MinPtsPerLine = 40;
                    //shdl3.MaxLines = 1;
                    //shdl3.SegmentationType = CProHough.SegmentationTypeEnum.SegmentationEdge;
                    //shdl3.AngleResolution = 1;

                    //shdl3.EdgeStrengthMode = CProHough.EdgeStrengthModeEnum.EdgeStrengthManual;
                    //shdl3.EdgeStrength = 30;
                    //shdl3.SmoothFilterSize = CProHough.SmoothFilterSizeEnum.Smooth5x5;
                    //shdl3.AngleVicinity = 1;
                    //shdl3.RadialDistanceResolution = 1; shdl3.RadialDistanceVicinity = 1;
                    //shdl3.StartAngle = 0;
                    //shdl3.EndAngle = 180;
                    //shdl3.Execute(img);
                    pass_fail[i] = true;
                }
                if (cmbxItem == "SHDLRight")
                {

                    //preproc.Equalize(testimg, testimg, 0, 101, 0, 255);
                    //preproc.Thresh(testimg, testimg, 82, 255, false,CProBasic.ThreshLevelType.ThreshLevelDouble,CProBasic.ThreshResultType.ThreshResultBinary);
                    //Mat temp = new Mat(testimg.Height, testimg.Width, MatType.CV_8UC1, testimg.GetData());
                    //Mat temp3 = new Mat(temp, new Rect(rectRois[cmbxItem].x, rectRois[cmbxItem].y, rectRois[cmbxItem].width, rectRois[cmbxItem].height));
                    //Mat dest = new Mat();
                    //Cv2.Reduce(temp3.Clone(), dest, ReduceDimension.Row, ReduceTypes.Sum, MatType.CV_32SC1);
                    //Mat temp2 = new Mat();
                    //Cv2.Repeat(dest, temp3.Height, 1, temp2);
                    //Cv2.Normalize(temp2, temp2, 0, 255, NormTypes.MinMax);
                    //temp2.ConvertTo(temp3, MatType.CV_8UC1);
                    ////using (new Window("in", WindowMode.Normal, temp2)) ;
                    //// using (new Window("proj", WindowMode.Normal, temp)) ;
                    ////Cv2.ImWrite("testline.bmp", temp);
                    //CProImage img = new CProImage(temp.Width, temp.Height, CProData.FormatEnum.FormatUByte, rectRois[cmbxItem], temp.Data, true);
                    //shdl4.MinPtsPerLine = 40;
                    //shdl4.MaxLines = 1;
                    //shdl4.SegmentationType = CProHough.SegmentationTypeEnum.SegmentationEdge;
                    //shdl4.AngleResolution = 1;

                    //shdl4.EdgeStrengthMode = CProHough.EdgeStrengthModeEnum.EdgeStrengthManual;
                    //shdl4.EdgeStrength = 20;
                    //shdl4.SmoothFilterSize = CProHough.SmoothFilterSizeEnum.Smooth5x5;
                    //shdl4.AngleVicinity = 1;
                    //shdl4.RadialDistanceResolution = 1; shdl4.RadialDistanceVicinity = 1;
                    //shdl4.StartAngle = 0;
                    //shdl4.EndAngle = 180;
                    //shdl4.Execute(img);
                    pass_fail[i] = true;
                    //if (shdl1.GetResultNumLines() == 1 && shdl2.GetResultNumLines() == 1 && shdl3.GetResultNumLines() == 1 && shdl4.GetResultNumLines() == 1)
                    //{
                    //    shdlhg.Text = Math.Sqrt(Math.Pow((shdl1.GetResultIntersectPoint1(0).x - shdl2.GetResultIntersectPoint1(0).x), 2) + Math.Pow((shdl1.GetResultIntersectPoint1(0).y - shdl2.GetResultIntersectPoint1(0).y), 2)).ToString();
                    //    shdlwd.Text = Math.Sqrt(Math.Pow((shdl3.GetResultIntersectPoint1(0).x - shdl4.GetResultIntersectPoint1(0).x), 2) + Math.Pow((shdl3.GetResultIntersectPoint1(0).y - shdl4.GetResultIntersectPoint1(0).y), 2)).ToString();
                    //    lh = Math.Sqrt(Math.Pow((shdl1.GetResultIntersectPoint1(0).x - shdl2.GetResultIntersectPoint1(0).x), 2) + Math.Pow((shdl1.GetResultIntersectPoint1(0).y - shdl2.GetResultIntersectPoint1(0).y), 2));
                    //    lw = Math.Sqrt(Math.Pow((shdl3.GetResultIntersectPoint1(0).x - shdl4.GetResultIntersectPoint1(0).x), 2) + Math.Pow((shdl3.GetResultIntersectPoint1(0).y - shdl4.GetResultIntersectPoint1(0).y), 2));
                    //    pass_fail[i] = (lh > 10) && (lw > 10);

                    //}

                }
               
                    
               
                
            }

            for (int i = 0; i < cmbBx_model.Items.Count; i++)
            {
                tot_result = tot_result && pass_fail[i];
            }
            // To pass the values to csv file
            passlead = pass_fail[7] && pass_fail[8];
            passabut = pass_fail[9] && pass_fail[13] && pass_fail[14];
            passpaul = pass_fail[11] && pass_fail[15] && pass_fail[16];
            passrivet = pass_fail[17] && pass_fail[18];
            //passsizer = pass_fail[21] && pass_fail[22] && pass_fail[23] && pass_fail[24];
            passsizer = pass_fail[31];
            passclr = pass_fail[29] && pass_fail[32];
            //passsizel = pass_fail[25] && pass_fail[26] && pass_fail[27] && pass_fail[28];
            passsizel = pass_fail[30];
            COUNTS++;
            // To insert the new output
            string time = DateTime.Now.ToString("HH:mm:ss tt");
            string Date = DateTime.Now.ToString("yyyy-MM-dd");
            string appendText = name + de1 + Date + de1 + time + de1 + pass_fail[0].ToString() + de1 + passclr.ToString() + de1 + pass_fail[1].ToString() + de1 + pass_fail[2].ToString() + de1 + passsizel.ToString() + de1 + pass_fail[31].ToString() + de1 + pass_fail[3].ToString() + de1 + passsizer.ToString() +
                    de1 + pass_fail[30].ToString() + de1 + pass_fail[4].ToString() + de1 + pass_fail[5].ToString() + de1 + pass_fail[6].ToString() + de1 + passlead.ToString() + de1 + passabut.ToString() + de1 +
                     pass_fail[10].ToString() + de1 + passpaul.ToString() + de1 + pass_fail[19].ToString() + de1 + pass_fail[12].ToString() + de1 + passrivet.ToString() + de1 + tot_result.ToString() + de1 + Environment.NewLine;
            File.AppendAllText(n, appendText);
            tlStrpPrgsBar1.Value = 80; float p; float k; 
            // perRline.LinePerpendicular((float)1.5708, centerPoint, out p, out k);
            if (!tot_result)
            {
                
                fail += 1;
                lblFailed.Text = fail.ToString();
                tot += 1;
                lblTotal.Text = tot.ToString();
                lblPassed.Text = pass.ToString();
                lbl_pass_fail.Text = "Fail";
                total = tot;
                failed = fail;
              //iomodule.WriteSingleCoil(1284, true);
                lbl_pass_fail.BackColor = Color.Red;
                passtext = lbl_pass_fail.Text;

                //if (img_save)
                //{
                //    if ((Form2.text2) == "LH MODEL1")
                //        Cv2.ImWrite(@"D:\LHS output file\B562\IMAGE\" + foldername + "\\" + COUNTS.ToString() + ".bmp", img);
                //    if ((Form2.text2) == "LH MODEL2")
                //        Cv2.ImWrite(@"D:\LHS output file\B299\IMAGE\" + foldername + "\\" + COUNTS.ToString() + ".bmp", img);
                //    if ((Form2.text2) == "RH MODEL1")
                //        Cv2.ImWrite(@"D:\RHS output file\B562\IMAGE\" + foldername + "\\" + COUNTS.ToString() + ".bmp", img);
                //    if ((Form2.text2) == "RH MODEL2")
                //        Cv2.ImWrite(@"D:\RHS output file\B299\IMAGE\" + foldername + "\\" + COUNTS.ToString() + ".bmp", img);
                //}
                //To close the already opened FORM3
                if (Application.OpenForms.OfType<Form3>().Count() == 1)
                    Application.OpenForms.OfType<Form3>().First().Close();
                Form3 st = new Form3();
                //st.Show();
            }
            else
            {
                //fail += 1;
                lblFailed.Text = fail.ToString();
                tot += 1;
                lblTotal.Text = tot.ToString();
                pass += 1;
                lblPassed.Text = pass.ToString();
                lbl_pass_fail.Text = "Pass";
               //iomodule.WriteSingleCoil(1283, true);
                total = tot;
                passed = pass;
                lbl_pass_fail.BackColor = Color.LightGreen;
                passtext = lbl_pass_fail.Text;
                //To close the already opened FORM3 
                if (Application.OpenForms.OfType<Form3>().Count() == 1)
                    Application.OpenForms.OfType<Form3>().First().Close();
                Form3 st = new Form3();
                //st.Show();
            }
           tlStrpPrgsBar1.Value = 100;
        }
        private CProPoint[] get_line_pts(String parent,String child, CProImage img_mes)
        {
            CProPoint[] pts = new CProPoint[2];
            return pts;
        }
        private void FitWindow()
        {
            // Fit form to image size
            Rectangle windowRect = new Rectangle();
            Rectangle screenRect = Screen.AllScreens[0].WorkingArea;

            //Set MinimumSize and MaximumSize in order to allow resizing
            this.MinimumSize = new System.Drawing.Size(0, 0);
            this.MaximumSize = new System.Drawing.Size(screenRect.Width, screenRect.Height);

            //Resize the form and the picture box 
            img_disp.Size = new System.Drawing.Size(Image.Width, Image.Height);
            windowRect.Width = img_disp.Left + img_disp.Width;
            windowRect.Height = img_disp.Top + img_disp.Height;
            this.ClientSize = new System.Drawing.Size(windowRect.Width, windowRect.Height);

            //Set maximumsize in order to forbidden resizing
            this.MaximumSize = this.Size;
            this.MinimumSize = this.Size;

            this.Refresh();
        }

        private void process_image(Mat imgIn, bool offline = false)
        {
            tlStrpPrgsBar1.Value = 0;
            Mat img = new Mat();
            imgIn.CopyTo(imgInput);
            Cv2.CvtColor(imgIn, img, ColorConversionCodes.BGR2GRAY);
            if (cmbBx_model.SelectedItem.ToString() == "Spring1")
            {
                templ1.CopyTo(templl);
                templ2.CopyTo(templr);
                nonzpix = nonzpix1;
                plus_dis = plus_dis1;
                red_clrs = red_clrs1;
            }
            else if (cmbBx_model.SelectedItem.ToString() == "Spring2")
            {
                templ5.CopyTo(templl);
                templ6.CopyTo(templr);
                nonzpix = nonzpix1;
                plus_dis = plus_dis1;
                red_clrs = red_clrs1;
            }
            else if (cmbBx_model.SelectedItem.ToString() == "Spring3")
            {
                templ7.CopyTo(templl);
                templ8.CopyTo(templr);
                nonzpix = nonzpix1;
                plus_dis = plus_dis1;
                red_clrs = red_clrs1;
            }
            else if (cmbBx_model.SelectedItem.ToString() == "Spring4")
            {
                templ3.CopyTo(templl);
                templ4.CopyTo(templr);
                nonzpix = nonzpix2;
                plus_dis = plus_dis2;
                red_clrs = red_clrs2;
            }
            else {
                MessageBox.Show("No modle selected", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            Mat timg = new Mat();
            templl.CopyTo(timg);
            OpenCvSharp.Size ksize = new OpenCvSharp.Size(3, 3);
            //Cv2.Blur(timg,timg,ksize);
            int lowThreshold = 20, highThreshold = 80;
            int kernel_size = 3;
            //Cv2.Canny(timg,timg,lowThreshold, highThreshold);
            cal_grad(timg, timg);
            //Cv2.ImShow("ot", timg);
            //Cv2.WaitKey(0);
            Mat timg2 = new Mat();
            templr.CopyTo(timg2);
            cal_grad(timg2, timg2);
            //Cv2.Flip(timg, timg2, FlipMode.XY);
            //pictureBox1.Image = timg.ToBitmap();
            tlStrpPrgsBar1.Value = 20;
            //int match_method = 0;
            Mat img_display = new Mat();
            img.CopyTo(img_display);
            Mat result = new Mat();
            // Cv2.Blur(img, img, ksize);
            //Cv2.Canny(img, img, lowThreshold, highThreshold);
            cal_grad(img, img);


            /// Create the result matrix
            int result_cols = img.Cols - templl.Cols + 1;
            int result_rows = img.Rows - templl.Rows + 1;
            int result2_cols = img.Cols - templr.Cols + 1;
            int result2_rows = img.Rows - templr.Rows + 1;
            result.Create(result_cols, result_rows, MatType.CV_32FC1);
            Mat result2 = new Mat();
            result2.Create(result2_cols, result2_rows, MatType.CV_32FC1);
            /// Do the Matching and Normalize
            Cv2.MatchTemplate(img, timg, result, TemplateMatchModes.CCorrNormed);
            Cv2.MatchTemplate(img, timg2, result2, TemplateMatchModes.CCorrNormed);
            // Cv2.Normalize(result, result, 0, 1, NormTypes.MinMax, -1);
            /// Localizing the best match with minMaxLoc
            double minVal; double maxVal; OpenCvSharp.Point minLoc; OpenCvSharp.Point maxLoc;
            OpenCvSharp.Point matchLoc, res_loc;
            double maxVal2;
            tlStrpPrgsBar1.Value = 40;
            Cv2.MinMaxLoc(result, out minVal, out maxVal, out minLoc, out maxLoc);
            Cv2.MinMaxLoc(result2, out minVal, out maxVal2, out minLoc, out maxLoc);
            if (maxVal2 > maxVal)
            {
                Cv2.MinMaxLoc(result2, out minVal, out maxVal2, out minLoc, out maxLoc);
                matchLoc = maxLoc; //need to change based on method
            }
            else {
                Cv2.MinMaxLoc(result, out minVal, out maxVal, out minLoc, out maxLoc);
                matchLoc = maxLoc; //need to change based on method
            }
            
            res_loc = new OpenCvSharp.Point(matchLoc.X + templl.Cols, matchLoc.Y + templl.Rows);
        
            Mat cimg = new Mat();
            Mat gimg = new Mat();
            //Cv2.MedianBlur(imgInput, cimg, 3);
            imgInput.CopyTo(cimg);
            Mat img_lab = new Mat();
            //Cv2.ImShow("rgb",imgInput);
            Cv2.CvtColor(imgInput, img_lab, ColorConversionCodes.BGR2HSV);
            //Cv2.ImShow("lab",img_lab);
            Cv2.CvtColor(cimg, gimg, ColorConversionCodes.BGR2GRAY);
            CircleSegment[] clrs;
            clrs = Cv2.HoughCircles(gimg, HoughMethods.Gradient, 1, 100, 50, 40, 90, 180);
            Scalar scr = new Scalar(Bm, Gm, Rm);
            Scalar scr2 = new Scalar(Bx, Gx, Rx);
            Scalar scr3 = new Scalar(0, Gm, Rm);
            Scalar scr4 = new Scalar(15, Gx, Rx);
            Mat ot = new Mat();
            tlStrpPrgsBar1.Value = 60;
            Mat ot2 = new Mat();
            Cv2.InRange(img_lab, scr, scr2, ot);
            Cv2.InRange(img_lab, scr3, scr4, ot2);
            Cv2.InRange(img_lab, scr, scr2, ot);
            Cv2.BitwiseXor(ot, ot2, ot);
            OpenCvSharp.Size sz = new OpenCvSharp.Size(3, 3);
            Mat ele = new Mat(sz, MatType.CV_8UC1, 1);
            Cv2.MorphologyEx(ot, ot, MorphTypes.Open, ele);
            //Cv2.ImShow("ot",ot);
            //Cv2.WaitKey(0);
            int redclrs = 0;
            int[] ind = {0,0,0,0};
            for (int i = 0; i < clrs.Length; i++)
            {
                Mat temp = new Mat(ot.Rows, ot.Cols, MatType.CV_8UC1, 0);
                Cv2.Circle(temp, (int)clrs[i].Center.X, (int)clrs[i].Center.Y, (int)clrs[i].Radius, 255, -1);
                ot.CopyTo(temp, temp);
                if (Cv2.CountNonZero(temp) > nonzpix)
                {
                    ind[redclrs] = i;
                    redclrs += 1;
                }
            }
            tlStrpPrgsBar1.Value = 80;
            Scalar clr = new Scalar(255, 0, 0);
            if (redclrs == red_clrs)
            {
                    int i = ind[0];
                    Cv2.Circle(cimg, (int)clrs[i].Center.X, (int)clrs[i].Center.Y, 3, clr, 3);
                    Cv2.Circle(cimg, (int)clrs[i].Center.X, (int)clrs[i].Center.Y, (int)clrs[i].Radius, clr, 3);
                double dis = 0;
                if (red_clrs == 2)
                {
                    int j = ind[1];
                    Cv2.Circle(cimg, (int)clrs[j].Center.X, (int)clrs[j].Center.Y, (int)clrs[j].Radius, clr, 3);
                    Cv2.Circle(cimg, (int)clrs[j].Center.X, (int)clrs[j].Center.Y, 3, clr, 3);
                    dis = Math.Abs((clrs[j].Center.Y - clrs[i].Center.Y) * matchLoc.X - (clrs[j].Center.X - clrs[i].Center.X) * matchLoc.Y + clrs[j].Center.X * clrs[i].Center.Y - clrs[j].Center.Y * clrs[i].Center.X);
                    dis /= Math.Sqrt(Math.Pow((clrs[j].Center.Y - clrs[i].Center.Y), 2) + Math.Pow((clrs[j].Center.X - clrs[i].Center.X), 2));
                    //double m = (clrs[j].Center.Y - clrs[i].Center.Y) / (clrs[j].Center.X - clrs[i].Center.X + 0.0000001);
                }
                else
                {
                    dis = Math.Sqrt(Math.Pow((matchLoc.X - clrs[i].Center.X), 2) + Math.Pow((matchLoc.Y - clrs[i].Center.Y), 2));
                }
                if (dis > plus_dis)
                {
                        fail += 1;
                        lblFailed.Text = fail.ToString();
                        tot += 1;
                        lblTotal.Text = tot.ToString();
                        lblPassed.Text = pass.ToString();
                        lbl_pass_fail.Text = "Fail";
                        lbl_pass_fail.BackColor = Color.Red;
                        //iomodule.WriteSingleCoil(1280, true);
                        //iomodule.WriteSingleCoil(1281, false);
                    //iomodule.WriteSingleCoil(1282, false);
                }
                    else
                    {
                        //fail += 1;
                        lblFailed.Text = fail.ToString();
                        tot += 1;
                        lblTotal.Text = tot.ToString();
                        pass += 1;
                        lblPassed.Text = pass.ToString();
                        lbl_pass_fail.Text = "Pass";
                        lbl_pass_fail.BackColor = Color.LightGreen;
                        //iomodule.WriteSingleCoil(1281, true);
                        //iomodule.WriteSingleCoil(1280, false);
                    //iomodule.WriteSingleCoil(1282, true);
                }
                    
                }
                else
                {
                    fail += 1;
                    lblFailed.Text = fail.ToString();
                    tot += 1;
                    lblTotal.Text = tot.ToString();
                    lblPassed.Text = pass.ToString();
                    lbl_pass_fail.Text = "Fail";
                    lbl_pass_fail.BackColor = Color.Red;
                    //iomodule.WriteSingleCoil(1280, true);
                    //iomodule.WriteSingleCoil(1281, false);
                //iomodule.WriteSingleCoil(1282, false);
            }

            /// Show me what you got
            Scalar clr2 = new Scalar(0, 255, 0);
            Cv2.Rectangle(cimg, matchLoc, res_loc, clr2, 2);
            img_disp.Image = cimg.ToBitmap();
            tlStrpPrgsBar1.Value = 100;
        }

        private void cal_grad(Mat iimg, Mat oimg)
        {
            Mat img = new Mat();
            iimg.CopyTo(img);
            Mat grad_x = new Mat();
            Mat grad_y = new Mat() ;
            double[] g = new double[]{ 0.1353, 0.6065, 0, 0.6065, 0.1353 };
            Mat g_mat = new Mat(5, 1, MatType.CV_64FC1, g);
            double[] d = new double[] { 0.2707, 0.6065, 0, -0.6065, -0.2707 };
            Mat d_mat = new Mat(5, 1, MatType.CV_64FC1, d);
            Cv2.SepFilter2D(img, grad_x, MatType.CV_64FC1, d_mat, g_mat);
            Cv2.SepFilter2D(img, grad_y, MatType.CV_64FC1, g_mat, d_mat);
            Mat grad = new Mat();
            Mat x_sqr = new Mat();
            Mat y_sqr = new Mat();
            Cv2.Pow(grad_x, 2, x_sqr);
            Cv2.Pow(grad_y, 2, y_sqr);
            Cv2.Pow(x_sqr + y_sqr, 0.5, grad);
            grad.ConvertTo(oimg, MatType.CV_8UC1);
        }

        public void btnStop_Click(object sender, EventArgs e)
        {
            btnrun = false;
            ckBox_SH.Enabled = true;
            ckBox_ST.Enabled = true;
            //ckBox_ET.Enabled = true;
            btnStart.Enabled = true;
            try
            {
                usbcam.StreamGrabber.Stop();
            }
            catch (Exception exception)
            {
                throw(exception);
            }

        }

        private void ckBox_ST_CheckedChanged(object sender, EventArgs e)
        {
            if (ckBox_ST.Checked)
            {
                extTrig = false;
                sftTrig = true;
                ckBox_ET.Checked = false;
                usbcam.Parameters[PLCamera.TriggerSource].TrySetValue(PLCamera.TriggerSource.Software);
                usbcam.Parameters[PLCamera.TriggerMode].TrySetValue(PLCamera.TriggerMode.On);
                btnStart.Enabled = true;
            }
            else {
                usbcam.Parameters[PLCamera.TriggerMode].TrySetValue(PLCamera.TriggerMode.Off);
                btnStart.Enabled = false;
            }
        }

        private void ckBox_ET_CheckedChanged(object sender, EventArgs e)
        {
            if (ckBox_ET.Checked)
            {
                extTrig = true;
                sftTrig = false;
                ckBox_ST.Checked = false;
                usbcam.Parameters[PLCamera.TriggerSource].TrySetValue(PLCamera.TriggerSource.Line1);
                usbcam.Parameters[PLCamera.TriggerMode].TrySetValue(PLCamera.TriggerMode.On);
                btnStart.Enabled = true;
            }
            else {
                usbcam.Parameters[PLCamera.TriggerMode].TrySetValue(PLCamera.TriggerMode.Off);
                btnStart.Enabled = false;
            }

        }

        private void ckBox_SH_CheckedChanged(object sender, EventArgs e)
        {
            if (ckBox_SH.Checked)
            {
                shEnble = true;
                if (shEnble)
                {
                    txtBox.Text += "Connecting.....\r\n";
                    tcpclnt.Connect("192.168.0.100", 1024);
                    // use the ipaddress as in the server program

                    txtBox.Text += "Connected \r\n";
                    stm = tcpclnt.GetStream();
                }
            }
        }

        private void basler_gige_grab_FormClosing(object sender, FormClosingEventArgs e)
        {
            tcpclnt.Close();
            /* Release the buffer. */

            if (usbcam != null)
            {
                usbcam.Close();
                usbcam.Dispose();
            }

            txtBox.Text += "\nPress enter to exit.\r\n";

            /* Shut down the pylon runtime system. Don't call any pylon method after*/
        }

        delegate void SetTextCallback(string text);

        private void SetText(string text)
        {
            // InvokeRequired required compares the thread ID of the
            // calling thread to the thread ID of the creating thread.
            // If these threads are different, it returns true.
            if (this.txtBox1.InvokeRequired)
            {
                SetTextCallback d = new SetTextCallback(SetText);
                this.Invoke(d, new object[] { text });
            }
            else
            {
                this.txtBox1.Text = text;
            }
        }

        private void button1_Click(object sender, EventArgs e)
        {
            folderBrowserDialog1.Description = "Select Folder to Log Images";
            folderBrowserDialog1.ShowDialog();
            if (folderBrowserDialog1.SelectedPath == "")
            {
                MessageBox.Show("Images won't be logged", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            img_save = true;
        }

        private void btnSave_Click(object sender, EventArgs e)
        {
           
            draw = false;
            draw_roi = false;
            saveFileDialog1.ShowDialog();
            if(saveFileDialog1.FileName == "")
            {
                MessageBox.Show("No file saved", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            txt_save_file.Text = saveFileDialog1.FileName; 
            FileStorage fs = new FileStorage(txt_save_file.Text, FileStorage.Mode.Write);
            int itemCount = cmbBx_model.Items.Count;
            for (int i = 0; i < itemCount; i++)
            {
                string cmbxItem = cmbBx_model.GetItemText(cmbBx_model.Items[i]);
                
               
                    fs.Write("enblEdge" + cmbxItem, enableSearchEdge[i] ? 1 : 0);
                    fs.Write("enblArea" + cmbxItem, enableSearchArea[i] ? 1 : 0);
                    fs.Write("enblHoughCircle" + cmbxItem, enableHoughCircle[i] ? 1 : 0);

                if ((Form2.text1) == "DRUM BRAKE - FORD L.H")
                {
                    if (enableSearchEdge[i])
                    {
                        edgeSearches[cmbxItem].Save("searchedgeLH_data" + cmbxItem);
                    }
                    else if (enableSearchArea[i])
                    {
                        areaSearches[cmbxItem].Save("searchareaLH_data" + cmbxItem);
                    }
                }
                else
                {
                    if (enableSearchEdge[i])
                    {
                        edgeSearches[cmbxItem].Save("searchedge_data" + cmbxItem);
                    }
                    else if (enableSearchArea[i])
                    {
                        areaSearches[cmbxItem].Save("searcharea_data" + cmbxItem);
                    }
                }
                fs.Write("roi" + cmbxItem + "x", rectRois[cmbxItem].x);
                fs.Write("roi" + cmbxItem + "y", rectRois[cmbxItem].y);
                fs.Write("roi" + cmbxItem + "w", rectRois[cmbxItem].width);
                fs.Write("roi" + cmbxItem + "h", rectRois[cmbxItem].height);
                fs.Write("enblEdgeDetect" + cmbxItem, enableEdgeDetect[i] ? 1 : 0);
                fs.Write("enblEqualize" + cmbxItem, enableEqualize[i] ? 1 : 0);
                fs.Write("enblHistEqlze" + cmbxItem, enableHistEqlze[i] ? 1 : 0);
                fs.Write("enblThresh" + cmbxItem, enableThreshold[i] ? 1 : 0);
                fs.Write("srcLow" + cmbxItem, srcLow[i]);
                fs.Write("srcHigh" + cmbxItem, srcHigh[i]);
                fs.Write("dstLow" + cmbxItem, dstLow[i]);
                fs.Write("dstHigh" + cmbxItem, dstHigh[i]);
                fs.Write("edgeThresh" + cmbxItem, edgeThresh[i]);
                fs.Write("threshlo" + cmbxItem, threshlo[i]);
                fs.Write("threshhi" + cmbxItem, threshhi[i]);
                fs.Write("threshinv" + cmbxItem, threshinv[i] ? 1 : 0);
                
            }
            string mdl = cmbBx_model.SelectedItem.ToString();
            fs.Write("model", mdl);
            fs.Write("templ1", templ1);
            fs.Write("templ2", templ2);
            fs.Write("templ3", templ3);
            fs.Write("templ4", templ4);
            fs.Write("templ5", templ5);
            fs.Write("templ6", templ6);
            fs.Write("templ7", templ7);
            fs.Write("templ8", templ8);
            fs.Write("nonzpixc1", nonzpix1);
            fs.Write("redclrs1",red_clrs1);
            fs.Write("plusdis1", plus_dis1);
            fs.Write("nonzpixc2", nonzpix2);
            fs.Write("redclrs2", red_clrs2);
            fs.Write("plusdis2", plus_dis2);
            fs.Write("Rm", Rm);
            fs.Write("Gm", Gm);
            fs.Write("Bm", Bm);
            fs.Write("Rx", Rx);
            fs.Write("Gx", Gx);
            fs.Write("Bx", Bx);
            fs.Write("expo_600", expo_600);
            fs.Write("expo_1100", expo_1100);
            fs.Write("delay_600", delay_600);
            fs.Write("delay_1100", delay_1100);
            fs.Write("EnblAdmin", 0);
            fs.Write("modified", DateTime.Now.ToString());
            fs.Release();
        }

        private void btn_browse_Click(object sender, EventArgs e)
        {
            //openFileDialog1.Description = "Select Folder to Log Images";
            draw = false;
            draw_roi = false;
            openFileDialog1.ShowDialog();
            if (openFileDialog1.FileName == "")
            {
                MessageBox.Show("No file selected", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            txtImgFile.Text = openFileDialog1.FileName;
            if (!templ1.Empty() && !templ2.Empty() && !templ3.Empty() && !templ4.Empty() && openFileDialog1.FileName != "" && !templ5.Empty() && !templ6.Empty() && !templ7.Empty() && !templ8.Empty())
            {
                btn_test.Enabled = true;
            }
            else
            { btn_test.Enabled = false; }
            if(openFileDialog1.FileName != "")
            {
                btn_train_minus.Enabled = true;
                btn_train_plus.Enabled = true;
                FitImg.Enabled = true;
            }
        }

        private void btn_load_Click(object sender, EventArgs e)
        {

            draw = false;
            draw_roi = false;
            btnSave.Enabled = true;
            openFileDialog2.ShowDialog();
            if (openFileDialog2.FileName == "")
            {
                MessageBox.Show("No File selected", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            txt_save_file.Text = openFileDialog2.FileName;
         
             Properties.Settings.Default.sol_file1 = txt_save_file.Text;
            Properties.Settings.Default.Save();
            load_settings();
        }
        private void load_settings()
        {
            draw = false;
            draw_roi = false;
            btnSave.Enabled = true;

            if ((Form2.text1) == "DRUM BRAKE - FORD R.H")
            {
                String sol_file1 =Properties.Settings.Default.sol_file1;
                txt_save_file.Text = sol_file1;
                if ((Form2.text2) == "RH MODEL1")
                {
                    if (!Directory.Exists(@"E:\RHS output file\B562\"))
                    {
                        Directory.CreateDirectory(@"E:\RHS output file\B562\");
                    }
                    n = string.Format(@"E:\RHS output file\B562\DATA for RHS-B562-{0:yyyy-MM-dd}.csv", DateTime.Now);//output result storage path for RHS
                    if (!Directory.Exists(@"E:\RHS output file\B562\IMAGE\"+foldername))
                    {
                        Directory.CreateDirectory(@"E:\RHS output file\B562\IMAGE\" + foldername);
                    }
                }
                else
                {
                    if (!Directory.Exists(@"E:\RHS output file\B299\"))
                    {
                        Directory.CreateDirectory(@"E:\RHS output file\B299\");
                    }
                    n = string.Format(@"E:\RHS output file\B299\DATA for RHS-B299-{0:yyyy-MM-dd}.csv", DateTime.Now);//output result storage path for RHS
                    if (!Directory.Exists(@"E:\RHS output file\B299\IMAGE\" + foldername))
                    {
                        Directory.CreateDirectory(@"E:\RHS output file\B299\IMAGE\" + foldername);
                    }
                }
            }

            else
            {
                
                String sol_file_lhs = Properties.Settings.Default.sol_file_lhs;
                txt_save_file.Text = sol_file_lhs;
                if ((Form2.text2) == "LH MODEL1")
                {
                    if (!Directory.Exists(@"E:\LHS output file\B562\"))
                    {
                        Directory.CreateDirectory(@"E:\LHS output file\B562\");
                    }
                    n = string.Format(@"E:\LHS output file\B562\DATA for LHS-B562-{0:yyyy-MM-dd}.csv", DateTime.Now);//output result storage path for LHS
                   
                    if (!Directory.Exists(@"E:\LHS output file\B562\IMAGE\" + foldername))
                    {
                        Directory.CreateDirectory(@"E:\LHS output file\B562\IMAGE\" + foldername);
                    }
      
                }
                else
                {
                    if (!Directory.Exists(@"E:\LHS output file\B299\"))
                    {
                        Directory.CreateDirectory(@"E:\LHS output file\B299\");
                    }
                    n = string.Format(@"E:\LHS output file\B299\DATA for LHS-B299-{0:yyyy-MM-dd}.csv", DateTime.Now);//output result storage path for LHS
                    if (!Directory.Exists(@"E:\LHS output file\B299\IMAGE\" + foldername))
                    {
                        Directory.CreateDirectory(@"E:\LHS output file\B299\IMAGE\" + foldername);
                    }
                }
            }
            if (!File.Exists(n))
            {
                // Create a file to write to.//To insert the first output of the day
                string createText = "P01   -GUIDE PLATE PRESENCE/POSITION" + de1 + de1 + de1 + "P11   -ROLLPIN-2 PRESENCE" + Environment.NewLine + "P02   -LINING GRADE/COLOR" + de1 + de1 + de1 + "P12   -PUSHROD PRESENCE/POSITION" + Environment.NewLine + "P03   -LEVER HAND PRESENCE/POSITION" + de1 + de1 + de1 + "P13   -SHOE LEADING/TRAILING" + Environment.NewLine +
                "P04   -SHD LH CLIP ORIENTATION" + de1 + de1 + de1 + "P14   -ABUTMENTSPRING PRESENCE/FITMENT" + Environment.NewLine + "P05   -SHD LH CLIP SIZE " + de1 + de1 + de1 + "P15   -WCSPRING PRESENCE/FITMENT" + Environment.NewLine + "P06   -SHD LH CLIP POSITION" + de1 + de1 + de1 + "P16   -PAULSPRING PRESENCE/FITMENT" + Environment.NewLine + "P07   -SHD RH CLIP ORIENTATION" +
                de1 + de1 + de1 + "P17   -BACK PLATE DIAMETER" + Environment.NewLine + "P08   -SHD RH CLIP SIZE" + de1 + de1 + de1 + "P18   -LEVER NOTCH PRESENCE" + Environment.NewLine + "P09   -SHD RH CLIP POSITION" + de1 + de1 + de1 + "P19   -ABUTMENT RIVET DIAMETER" + Environment.NewLine + "P10   -ROLLPIN-1 PRESENCE" + Environment.NewLine +
                 "S.No" + de1 + "DATE" + de1 + "TIME" + de1 + "P01" + de1 + "P02" + de1 + "P03" + de1 + "P04" + de1 + "P05" + de1 + "P06" + de1 + "P07" + de1 + "P08" + de1 + "P09" + de1 + "P10" + de1 + "P11" + de1 + "P12" + de1 +
               "P13" + de1 + "P14" + de1 + "P15" + de1 + "P16" + de1 + "P17" + de1 + "P18" + de1 + "P19" + de1 + "RESULT" + de1 + Environment.NewLine + "0" + Environment.NewLine;
                File.WriteAllText(n, createText);
            }
            else
            {
                var lastLine = File.ReadLines(n).Last();
                var values = lastLine.Split(',');
                COUNTS = Convert.ToInt32(values[0]);
            }
            FileStorage fs = new FileStorage(txt_save_file.Text, FileStorage.Mode.Read);
            int itemCount = cmbBx_model.Items.Count;
            enableSearchEdge = new bool[itemCount];
            enableEqualize = new bool[itemCount];
            enableEdgeDetect = new bool[itemCount];
            enableHistEqlze = new bool[itemCount];
            enableThreshold = new bool[itemCount];
            enableHoughCircle = new bool[itemCount];
            enableSearchArea = new bool[itemCount];
            srcLow = new float[itemCount];
            srcHigh = new float[itemCount];
            dstLow = new float[itemCount];
            dstHigh = new float[itemCount];
            edgeThresh = new int[itemCount];
            threshlo = new float[itemCount];
            threshhi = new float[itemCount];
            threshinv = new bool[itemCount];
            pass_fail = new bool[itemCount];
            for (int i = 0; i < itemCount; i++)
            {
                string cmbxItem = cmbBx_model.GetItemText(cmbBx_model.Items[i]);
                enableSearchEdge[i] = fs["enblEdge" + cmbxItem].ReadInt() == 1;
                enableSearchArea[i] = fs["enblArea" + cmbxItem].ReadInt() == 1;
                enableHoughCircle[i] = fs["enblHoughCircle" + cmbxItem].ReadInt() == 1;
                if ((Form2.text1) == "DRUM BRAKE - FORD L.H")
                {
                   
                    if (enableSearchEdge[i] && !edgeSearches.ContainsKey(cmbxItem))
                    {
                        edgeSearches.Add(cmbxItem, new CProSearchEdge());
                        edgeSearches[cmbxItem].Load("searchedgeLH_data" + cmbxItem);
                    }
                    else if (enableSearchArea[i] && !areaSearches.ContainsKey(cmbxItem))
                    {
                        areaSearches.Add(cmbxItem, new CProSearchArea());
                        areaSearches[cmbxItem].Load("searchareaLH_data" + cmbxItem);
                    }
                    if (enableHoughCircle[i] && !HoughCircles.ContainsKey(cmbxItem))
                    {
                        HoughCircles.Add(cmbxItem, new CProHoughCircle());
                    }
                }
                else
                {
                    
                    if (enableSearchEdge[i] && !edgeSearches.ContainsKey(cmbxItem))
                    {
                        edgeSearches.Add(cmbxItem, new CProSearchEdge());
                        edgeSearches[cmbxItem].Load("searchedge_data" + cmbxItem);
                    }
                    else if (enableSearchArea[i] && !areaSearches.ContainsKey(cmbxItem))
                    {
                        areaSearches.Add(cmbxItem, new CProSearchArea());
                        areaSearches[cmbxItem].Load("searcharea_data" + cmbxItem);
                    }
                    if (enableHoughCircle[i] && !HoughCircles.ContainsKey(cmbxItem))
                    {
                        HoughCircles.Add(cmbxItem, new CProHoughCircle());
                    }
                }
                CProRect tempRoi = new CProRect();
                tempRoi.x = fs["roi" + cmbxItem + "x"].ReadInt();
                tempRoi.y = fs["roi" + cmbxItem + "y"].ReadInt();
                tempRoi.width = fs["roi" + cmbxItem + "w"].ReadInt();
                tempRoi.height = fs["roi" + cmbxItem + "h"].ReadInt();
                if(!rectRois.ContainsKey(cmbxItem))
                    rectRois.Add(cmbxItem, tempRoi);
                enableEdgeDetect[i] = fs["enblEdgeDetect" + cmbxItem].ReadInt() == 1;
                enableEqualize[i] = fs["enblEqualize" + cmbxItem].ReadInt() == 1;
                enableHistEqlze[i] = fs["enblHistEqlze" + cmbxItem].ReadInt() == 1;
                enableThreshold[i] = fs["enblThresh" + cmbxItem].ReadInt() == 1;
                
                srcLow[i] = fs["srcLow" + cmbxItem].ReadFloat();
                srcHigh[i] = fs["srcHigh" + cmbxItem].ReadFloat();
                dstLow[i] = fs["dstLow" + cmbxItem].ReadFloat();
                dstHigh[i] = fs["dstHigh" + cmbxItem].ReadFloat();
                edgeThresh[i] = fs["edgeThresh" + cmbxItem].ReadInt();
                threshlo[i] = fs["threshlo" + cmbxItem].ReadFloat();
                threshhi[i] = fs["threshhi" + cmbxItem].ReadFloat();
                threshinv[i] = fs["threshinv" + cmbxItem].ReadInt() == 1;
            }
            update_parameters();
            String modle = "";
            modle = fs["model"].ReadString();
            templ1 = fs["templ1"].ReadMat();
            templ2 = fs["templ2"].ReadMat();
            templ3 = fs["templ3"].ReadMat();
            templ4 = fs["templ4"].ReadMat();
            templ5 = fs["templ5"].ReadMat();
            templ6 = fs["templ6"].ReadMat();
            templ7 = fs["templ7"].ReadMat();
            templ8 = fs["templ8"].ReadMat();
            nonzpix1 = fs["nonzpixc1"].ReadInt();
            red_clrs1 =  fs["redclrs1"].ReadInt();
            plus_dis1 = fs["plusdis1"].ReadInt();
            nonzpix2 = fs["nonzpixc2"].ReadInt();
            red_clrs2 = fs["redclrs2"].ReadInt();
            plus_dis2 = fs["plusdis2"].ReadInt();
            Rm = fs["Rm"].ReadInt();
            Gm = fs["Gm"].ReadInt();
            Bm = fs["Bm"].ReadInt();
            Rx = fs["Rx"].ReadInt();
            Gx = fs["Gx"].ReadInt();
            Bx = fs["Bx"].ReadInt();
            expo_600 = fs["expo_600"].ReadDouble();
            expo_1100 = fs["expo_1100"].ReadDouble();
            delay_600 = fs["delay_600"].ReadInt();
            delay_1100 = fs["delay_1100"].ReadInt();
            enblAdmin = fs["EnblAdmin"].ReadInt();
            String saved="";
            saved = fs["modified"].ReadString();
            if (cmbBx_model.SelectedItem.ToString() == "Spring4")
            {
                if (btnStart.Enabled)
                    usbcam.Parameters[PLCamera.ExposureTime].TrySetValue(expo_1100);
                trig_delay = delay_1100;
            }
            else
            {
                if (btnStart.Enabled)
                    usbcam.Parameters[PLCamera.ExposureTime].TrySetValue(expo_600);
                trig_delay = delay_600;
            }
            fs.Release();
        }
        private void btn_train_plus_Click(object sender, EventArgs e)
        {
            draw = true;
            draw_roi = false;
            btn_test.Enabled = true;
            imgInput = new Mat(openFileDialog1.FileName, ImreadModes.Grayscale);
            Bitmap bitmap = BitmapConverter.ToBitmap(imgInput);
            //img_disp.ClientSize = new System.Drawing.Size(760, 480);
            img_disp.Image = bitmap;
            img_disp.SizeMode = PictureBoxSizeMode.AutoSize;
            temp_no = 1;
            //Mat opimg = new Mat(openFileDialog1.FileName, ImreadModes.GrayScale);
            //CProImage image_cpro = new CProImage();
            Image.Set(imgInput.Width, imgInput.Height, CProData.FormatEnum.FormatUByte, imgInput.Data, false);
            //if (!Image.Load(openFileDialog1.FileName))
            //    return;
        }
        private void img_disp_MouseDown(object sender, MouseEventArgs e)
        {
            if (draw || draw_roi)
            {
                IsMouseDown = true;
                StartLocation = new OpenCvSharp.Point(e.X, e.Y);
            }
        }

        private void img_disp_MouseUp(object sender, MouseEventArgs e)
        {
            if (draw || draw_roi)
            {
                if (IsMouseDown == true)
                {
                    EndLcation = new OpenCvSharp.Point(e.X, e.Y);
                    rect = GetRectangle();
                    IsMouseDown = false;
                    if (rect != null && rect.Width != 0 && rect.Height != 0)
                    {
                        double scaleX = (double)imgInput.Width / img_disp.ClientSize.Width;
                        double scaleY = (double)imgInput.Height / img_disp.ClientSize.Height;
                        Rect t = new Rect((int)(rect.X * scaleX), (int)(rect.Y * scaleY), (int)(rect.Width * scaleX), (int)(rect.Height * scaleY));
                        // Create model on image's ROI
                        Image.Roi = new CProRect(t.X, t.Y,t.Width, t.Height);
                        //preproc.HistEqualize(Image, Image);
                        Mat temp = new Mat(imgInput, t);
                        int search_ind = cmbBx_model.SelectedIndex;
                        string selected_item = cmbBx_model.SelectedItem.ToString();
                        if (draw)
                        {
                            if (enableHistEqlze[search_ind])
                            {
                                preproc.HistEqualize(Image, Image);
                            }
                            if (enableEqualize[search_ind])
                            {
                                preproc.Equalize(Image, Image, srcLow[i], srcHigh[i], dstLow[i], dstHigh[i]);
                            }
                            if (enableThreshold[i])
                            {
                                preproc.Thresh(Image, Image, threshlo[i], threshhi[i], threshinv[i], CProBasic.ThreshLevelType.ThreshLevelDouble, CProBasic.ThreshResultType.ThreshResultBinary);
                            }
                            if (enableEdgeDetect[search_ind])
                            {
                                CProImage tempimg = new CProImage(Image.Width, Image.Height, Image.Format, Image.Roi, Image.GetData(), true);
                                preproc.EdgeDetect(tempimg, Image, CProBasic.EdgeDetectType.EdgeDetectPrewitt, edgeThresh[i]);
                            }
                            if (enableSearchEdge[search_ind])
                            {
                                int is_present = edgeSearches[selected_item].FindModelByName(selected_item);
                                if (is_present != -1)
                                {
                                    edgeSearches[selected_item].RemoveModel(is_present);
                                }
                                edgeSearches[selected_item].AddModel(selected_item, Image);
                            }
                            if (enableSearchArea[search_ind])
                            {
                                int is_present = areaSearches[selected_item].FindModelByName(selected_item);
                                if (is_present != -1)
                                {
                                    areaSearches[selected_item].RemoveModel(is_present);
                                }
                                areaSearches[selected_item].AddModel(selected_item, Image);
                                is_present = areaSearches[selected_item].FindModelByName(selected_item);
                                areaSearches[selected_item].set_ModelMinRotation(is_present,-30);
                                areaSearches[selected_item].set_ModelMaxRotation(is_present, 30);
                            }


                        }
                        if(draw_roi)
                        {
                            rectRois[selected_item] = Image.Roi;
                        }
                        Bitmap bitmap = BitmapConverter.ToBitmap(temp);
                        pictureBox1.Image = bitmap;
                    }
                }
            }
           
        }


        private void img_disp_MouseMove(object sender, MouseEventArgs e)
        {
            if (draw || draw_roi)
            {
                if (IsMouseDown == true)
                {
                    EndLcation = new OpenCvSharp.Point(e.X, e.Y);
                    img_disp.Invalidate();
                }
            }
        }
        private Rectangle GetRectangle()
        {
            rect = new Rectangle();
            rect.X = Math.Min(StartLocation.X, EndLcation.X);
            rect.Y = Math.Min(StartLocation.Y, EndLcation.Y);
            rect.Width = Math.Abs(StartLocation.X - EndLcation.X);
            rect.Height = Math.Abs(StartLocation.Y - EndLcation.Y);

            return rect;
        }

        private void img_disp_Paint(object sender, PaintEventArgs e)
        {
            if (draw || draw_roi)
            {
                if (rect != null)
                {
                    e.Graphics.DrawRectangle(Pens.LightYellow, GetRectangle());
                }
            }
            if (drawres)
            {
                DrawResults(e.Graphics);
            }
        }
        private CProImage opencvfnprojection(CProImage testimg, CProRect rect, Boolean isVertical)
        {
            Mat temp = new Mat(testimg.Height, testimg.Width, MatType.CV_8UC1, testimg.GetData());
            //Cv2.ImWrite("testline2.bmp", temp);
            //Mat temp3 = new Mat(temp, new Rect(rect.x, rect.y, rect.width, rect.height));
            //Mat dest = new Mat();
            //Mat temp2 = new Mat();
            //if (isVertical)
            //{
            //    Cv2.Reduce(temp3.Clone(), dest, ReduceDimension.Row, ReduceTypes.Sum, MatType.CV_32SC1);
            //    Cv2.Repeat(dest, temp3.Height, 1, temp2);
            //}
            //else
            //{

            //    Cv2.Reduce(temp3.Clone(), dest, ReduceDimension.Column, ReduceTypes.Sum, MatType.CV_32SC1);
            //    Cv2.Repeat(dest, 1, temp3.Width, temp2);

            //}
            //Cv2.Normalize(temp2, temp2, 0, 255, NormTypes.MinMax);
            //temp2.ConvertTo(temp3, MatType.CV_8UC1);
            ////using (new Window("in", WindowMode.Normal, temp2));
            ////using (new Window("proj", WindowMode.Normal, temp));
            //Cv2.ImWrite("testline.bmp", temp);
            CProImage img = new CProImage(temp.Width, temp.Height, CProData.FormatEnum.FormatUByte, rect, temp.Data, true);
            return img;
        }
        private void update_display()
        {
            if (Image.Valid)
            {
                CProImage dispimg = new CProImage(Image.Width, Image.Height, Image.Format, Image.GetData(), true);
                CProImage shd = new CProImage();
                for (int j = 0; j < cmbBx_model.Items.Count; j++)
                {
                    CProImage ik = new CProImage();
                    string cmbxItem = cmbBx_model.GetItemText(cmbBx_model.Items[j]);
                    dispimg.Roi = rectRois[cmbxItem];
                    //if (cmbxItem == "SHDRTop")
                    //{
                    //    ik = new CProImage(dispimg.Width, dispimg.Height, CProData.FormatEnum.FormatUByte, rectRois["SHDR"], dispimg.GetData(), true);
                    //    shd.Set(dispimg.Width, dispimg.Height, CProData.FormatEnum.FormatUByte, rectRois["SHDR"], dispimg.GetData(), true);
                    //    preproc.Rotate(ik, shd, l1.GetResultAngle(0) - 90, CProBasic.RotateSizeOp.RotateSizeClip);
                    //    //preproc.Rotate(ik, shd, edgeSearches["SHDR"].GetMatchAngle(0) , CProBasic.RotateSizeOp.RotateSizeClip);
                    //    Mat rotate = new Mat(shd.Height, shd.Width, MatType.CV_8UC1, shd.GetData());
                    //    dispimg = opencvfnprojection(shd, rectRois[cmbxItem], false);
                    //}
                    //if (enableHistEqlze[j])
                    //{
                    //    preproc.HistEqualize(dispimg, dispimg);
                    //}
                    //if (enableEqualize[j])
                    //{
                    //    preproc.Equalize(dispimg, dispimg, srcLow[i], srcHigh[i], dstLow[i], dstHigh[i]);
                    //}
                    //if (enableThreshold[i])
                    //{
                    //    preproc.Thresh(dispimg, dispimg, threshlo[i], threshhi[i], threshinv[i], CProBasic.ThreshLevelType.ThreshLevelDouble, CProBasic.ThreshResultType.ThreshResultBinary);
                    //}
                    //if (enableEdgeDetect[j])
                    //{
                    //    CProImage tempimg = new CProImage(dispimg.Width, dispimg.Height, dispimg.Format, dispimg.Roi, dispimg.GetData(), true);
                    //    preproc.EdgeDetect(tempimg, dispimg, CProBasic.EdgeDetectType.EdgeDetectPrewitt, edgeThresh[i]);
                    //}
                    img_disp.Image = CDemoUtils.GetBitmapFromCProImage(dispimg);
                }
            }
        }

        private void DrawResults(Graphics g)
        {
            Pen pen = new Pen(Color.Green, 1);
            Pen pen2 = new Pen(Color.Blue, 2);
            SolidBrush brush = new SolidBrush(Color.Lime);
            Font Font = new Font("System", 10);

            // Declare local variables
            int I;
            float x0;
            float y0;
            int modelIndex;
            float angle;
            float scal;
            float x1;
            float y1;
            float x2;
            float y2;
            CProRect roi;
            float ca;
            float sa;
            float rx1;
            float ry1;
            float rx2;
            float ry2;
            float rx3;
            float ry3;
            float rx4;
            float ry4;
            string text;
            float scaleX = (float)img_disp.ClientSize.Width / imgInput.Width;
            float scaleY = (float)img_disp.ClientSize.Height/ imgInput.Height;
            CProImage dispimg = new CProImage(Image.Width, Image.Height, Image.Format, Image.GetData(), true);
            for (int j = 0; j < cmbBx_model.Items.Count; j++)
            { 
                string cmbxItem = cmbBx_model.GetItemText(cmbBx_model.Items[j]);
                if (!rectRois.ContainsKey(cmbxItem))
                    continue;
                g.DrawRectangle(pen2, rectRois[cmbxItem].x *scaleX, rectRois[cmbxItem].y*scaleY, rectRois[cmbxItem].width*scaleX, rectRois[cmbxItem].height*scaleY);
               
                if (enableSearchEdge[j])
                {
                    for (I = 0; I <= edgeSearches[cmbxItem].GetNumMatches() - 1; I++)
                    {
                        // Get current match information
                        x0 = edgeSearches[cmbxItem].GetMatchLocationX(I);
                        y0 = edgeSearches[cmbxItem].GetMatchLocationY(I);
                        modelIndex = edgeSearches[cmbxItem].GetMatchModelIndex(I);
                        angle = edgeSearches[cmbxItem].GetMatchAngle(I);
                        scal = edgeSearches[cmbxItem].GetMatchScale(I);

                        // Coordinates of the non-rotated rectangle

                        roi = edgeSearches[cmbxItem].get_ModelRoi(modelIndex);

                        x1 = x0 - (roi.width / 2) * scal;
                        y1 = y0 - (roi.height / 2) * scal;
                        x2 = x0 + (roi.width / 2) * scal;
                        y2 = y0 + (roi.height / 2) * scal;

                        // Coordinates of the rotated rectangle
                        ca = (float)Math.Cos(angle * RADIAN_PER_DEGREE);
                        sa = (float)Math.Sin(angle * RADIAN_PER_DEGREE);
                        rx1 = (x0 + (x1 - x0) * ca - (y1 - y0) * sa) * scaleX;
                        ry1 = (y0 + (x1 - x0) * sa + (y1 - y0) * ca) * scaleY;
                        rx2 = (x0 + (x2 - x0) * ca - (y1 - y0) * sa) * scaleX;
                        ry2 = (y0 + (x2 - x0) * sa + (y1 - y0) * ca) * scaleY;
                        rx3 = (x0 + (x2 - x0) * ca - (y2 - y0) * sa) * scaleX;
                        ry3 = (y0 + (x2 - x0) * sa + (y2 - y0) * ca) * scaleY;
                        rx4 = (x0 + (x1 - x0) * ca - (y2 - y0) * sa) * scaleX;
                        ry4 = (y0 + (x1 - x0) * sa + (y2 - y0) * ca) * scaleY;

                        // Draw the rectangle
                        g.DrawLine(pen, rx1, ry1, rx2, ry2);
                        g.DrawLine(pen, rx2, ry2, rx3, ry3);
                        g.DrawLine(pen, rx3, ry3, rx4, ry4);
                        g.DrawLine(pen, rx4, ry4, rx1, ry1);

                        // Draw the origin cross
                        g.DrawLine(pen, x0 * scaleX, (y0 - 20) * scaleY, x0 * scaleX, (y0 + 20) * scaleY);
                        g.DrawLine(pen, (x0 - 20) * scaleX, y0 * scaleY, (x0 + 20) * scaleX, y0 * scaleY);

                        // Draw match information
                        text = string.Format("{0} : {1}", I + 1, edgeSearches[cmbxItem].GetMatchModelName(I));
                        g.DrawString(text, Font, brush, scaleX * (x0 + 5), scaleY * y0);

                        // Draw more match information
                        //text += string.Format(" => Location: ({0:0.00}, {1:0.00}), Score: {2:0.00}, Angle: {3:0.00}, Scale: {4:0.00}", searches[j].GetMatchLocationX(I), searches[j].GetMatchLocationY(I), searches[j].GetMatchScore(I), searches[j].GetMatchAngle(I), searches[j].GetMatchScale(I));

                        //g.DrawString(text, Font, brush, 0, pictureBox1.Height - (searches[j].GetNumMatches() - I) * Font.Height);
                    }
                }
                else if(enableSearchArea[j])
                {
                    for (I = 0; I <= areaSearches[cmbxItem].GetNumMatches() - 1; I++)
                    {
                        // Get current match information
                        x0 = areaSearches[cmbxItem].GetMatchLocationX(I);
                        y0 = areaSearches[cmbxItem].GetMatchLocationY(I);
                        modelIndex = areaSearches[cmbxItem].GetMatchModelIndex(I);
                        angle = areaSearches[cmbxItem].GetMatchAngle(I);
                        scal = areaSearches[cmbxItem].GetMatchScale(I);

                        // Coordinates of the non-rotated rectangle

                        roi = areaSearches[cmbxItem].get_ModelRoi(modelIndex);

                        x1 = x0 - (roi.width / 2) * scal;
                        y1 = y0 - (roi.height / 2) * scal;
                        x2 = x0 + (roi.width / 2) * scal;
                        y2 = y0 + (roi.height / 2) * scal;

                        // Coordinates of the rotated rectangle
                        ca = (float)Math.Cos(angle * RADIAN_PER_DEGREE);
                        sa = (float)Math.Sin(angle * RADIAN_PER_DEGREE);
                        rx1 = (x0 + (x1 - x0) * ca - (y1 - y0) * sa) * scaleX;
                        ry1 = (y0 + (x1 - x0) * sa + (y1 - y0) * ca) * scaleY;
                        rx2 = (x0 + (x2 - x0) * ca - (y1 - y0) * sa) * scaleX;
                        ry2 = (y0 + (x2 - x0) * sa + (y1 - y0) * ca) * scaleY;
                        rx3 = (x0 + (x2 - x0) * ca - (y2 - y0) * sa) * scaleX;
                        ry3 = (y0 + (x2 - x0) * sa + (y2 - y0) * ca) * scaleY;
                        rx4 = (x0 + (x1 - x0) * ca - (y2 - y0) * sa) * scaleX;
                        ry4 = (y0 + (x1 - x0) * sa + (y2 - y0) * ca) * scaleY;

                        // Draw the rectangle
                        g.DrawLine(pen, rx1, ry1, rx2, ry2);
                        g.DrawLine(pen, rx2, ry2, rx3, ry3);
                        g.DrawLine(pen, rx3, ry3, rx4, ry4);
                        g.DrawLine(pen, rx4, ry4, rx1, ry1);

                        // Draw the origin cross
                        g.DrawLine(pen, x0 * scaleX, (y0 - 20) * scaleY, x0 * scaleX, (y0 + 20) * scaleY);
                        g.DrawLine(pen, (x0 - 20) * scaleX, y0 * scaleY, (x0 + 20) * scaleX, y0 * scaleY);

                        // Draw match information
                        text = string.Format("{0} : {1}", I + 1, areaSearches[cmbxItem].GetMatchModelName(I));
                        g.DrawString(text, Font, brush, scaleX * (x0 + 5), scaleY * y0);

                    }
                }
                if(enableHoughCircle[j])
                {
                    CProRect circle_roi = rectRois[cmbxItem];
                    // Display circles if there are any
                    for (int index = 0; index < HoughCircles[cmbxItem].GetResultNumCircles(); index++)
                    {
                        CProFPoint centerPoint = HoughCircles[cmbxItem].GetResultCenterPoint(index);
                        centerPoint.x = centerPoint.x + circle_roi.x;
                        centerPoint.y = centerPoint.y + circle_roi.y;
                        float radius = HoughCircles[cmbxItem].GetResultRadius(index);
                        g.DrawEllipse(pen, (centerPoint.x - radius)*scaleX, (centerPoint.y - radius)*scaleY, (radius * 2)*scaleX, (radius * 2)*scaleY);
                        g.DrawLine(pen, (centerPoint.x - 2)*scaleX, (centerPoint.y)*scaleY, scaleX*(centerPoint.x + 2), scaleY*centerPoint.y);
                        g.DrawLine(pen, centerPoint.x*scaleX, (centerPoint.y - 2)*scaleY, centerPoint.x*scaleX, (centerPoint.y + 2)*scaleY);
                        // Draw match information
                        text = string.Format("{0} : {1}",index+1,radius*scaleX );
                        g.DrawString(text, Font, brush, scaleX * (centerPoint.x), scaleY * centerPoint.y);
                        if ((j == 30)||(j==31))
                        {
                            if (j == 30)
                            {
                                float outang; float outdist; float ang = (float)((l1.GetResultAngle(0) * 1.5708) / 90);
                                perRline.LinePerpendicular(ang, centerPoint, out outang, out outdist);
                                g.DrawLine(pen, centerPoint.x, centerPoint.y, outdist, outang);
                                CProFPoint inter;
                                //intersectline.Line2LineIntersection(ang, k, 0, (float)getsubrpix, out inter);
                                // intersectline.Line2LineIntersection(ang, centerPoint.x, l1.GetResultAngle(0), (float)getsubrpix, out inter);
                                // g.DrawLine(Pens.Red, centerPoint.x, centerPoint.y, inter.x, inter.y);
                                float sectanglee; float outorgy;
                                segRline.Segment2Line(l1.GetResultIntersectPoint1(0), l1.GetResultIntersectPoint2(0), out sectanglee, out outorgy);
                                //intersectline.Line2LineIntersection(l1.GetResultAngle(0), orgy, 0, k, out inter);
                                intersectRline.Line2LineIntersection(ang, outorgy, outang, outdist, out inter);
                                float dist;
                                // pintersectline.
                                //distance.Point2PointDist(centerPoint, inter, out dist);
                                textBox3.Text = ((centerPoint.y) - (inter.y)).ToString();
                                g.DrawLine(Pens.Red, centerPoint.x, centerPoint.y, inter.x, inter.y);
                            }
                            else if(j==31)
                            {
                                float outang; float outdist; float ang = (float)((shdl1.GetResultAngle(0) * 1.5708) / 90);
                                perLline.LinePerpendicular(ang, centerPoint, out outang, out outdist);
                                g.DrawLine(pen, centerPoint.x, centerPoint.y, outdist, outang);
                                CProFPoint inter;
                                
                                float sectanglee; float outorgy;
                                segLline.Segment2Line(shdl1.GetResultIntersectPoint1(0), shdl1.GetResultIntersectPoint2(0), out sectanglee, out outorgy);

                                intersectLline.Line2LineIntersection(ang, outorgy, outang, outdist, out inter);
                                
                                textBox5.Text = ((centerPoint.y) - (inter.y)).ToString();
                                g.DrawLine(Pens.Red, centerPoint.x, centerPoint.y, inter.x, inter.y);
                            }
                            
                        }

                        //float h = (l1.GetResultIntersectPoint1(0).x);float kp = (float)h;
                        //intersectline.Point2LineDist(centerPoint, ang,l1.GetResultIntersectPoint1(0).x,l1.GetResultIntersectPoint1(0).y, out dist, out inter);
                    }
                }
                if (cmbxItem == "AssemblyDia")
                {
                   // CProRect croi = rectRois[cmbxItem];
                    CProFPoint center = spoke1.GetFittedCenter();
                    //center.x += croi.x;
                    //center.y += croi.y;
                    float radius = spoke1.GetFittedRadius();
                    g.DrawEllipse(pen, (center.x - radius) * scaleX, (center.y - radius) * scaleY, (radius * 2) * scaleX, (radius * 2) * scaleY);
                    // Draw match information
                    text = string.Format("{0} : {1}",  1, radius);
                    g.DrawString(text, Font, brush, scaleX * (center.x), scaleY * center.y);
                    for (int i = 0; i < spoke1.NumElement; i++)
                    {
                        CProFPoint stpt = spoke1.get_StartPointOfElement(i);
                        CProFPoint edpt = spoke1.get_EndPointOfElement(i);
                        CProVector pts = spoke1.get_EdgeList(i);
                        if (pts != null && pts.Size >= 1)
                        {
                            CProFPoint pt = (CProFPoint)pts.get_Data(0);

                            g.DrawEllipse(pen, pt.x * scaleX, pt.y * scaleY, 4, 4);
                        }
                        g.DrawLine(pen2, stpt.x * scaleX, stpt.y * scaleY, edpt.x * scaleX, edpt.y * scaleY);
                            
                    }
                }
                if (cmbxItem == "ShoeDia")
                {
                   
                }
                //if (cmbxItem == "SHDRTop" && (l1.GetResultNumLines() == 1))
                //{
                //    g.DrawLine(pen, l1.GetResultIntersectPoint1(0).x, l1.GetResultIntersectPoint1(0).y, l1.GetResultIntersectPoint2(0).x, l1.GetResultIntersectPoint2(0).y);
                   

                //}
                //if (cmbxItem == "SHDRBottom" && (l2.GetResultNumLines() == 1))
                //{
                //    g.DrawLine(Pens.Red, l2.GetResultIntersectPoint1(0).x, l2.GetResultIntersectPoint1(0).y, l2.GetResultIntersectPoint2(0).x, l2.GetResultIntersectPoint2(0).y);
                //    //  textBox2.Text = (l2.GetResultIntersectPoint1(0).y).ToString();
                //}
                //if (cmbxItem == "SHDRLeft" && (l3.GetResultNumLines() == 1))
                //{
                //    g.DrawLine(pen, l3.GetResultIntersectPoint1(0).x, l3.GetResultIntersectPoint1(0).y, l3.GetResultIntersectPoint2(0).x, l3.GetResultIntersectPoint2(0).y);
                //}
                //if (cmbxItem == "SHDRRight" && (l4.GetResultNumLines() == 1))
                //{
                //    g.DrawLine(pen, l4.GetResultIntersectPoint1(0).x, l4.GetResultIntersectPoint1(0).y, l4.GetResultIntersectPoint2(0).x, l4.GetResultIntersectPoint2(0).y);
                //}
                //if (cmbxItem == "SHDLTop" && (shdl1.GetResultNumLines() == 1))
                //{
                //    g.DrawLine(pen, shdl1.GetResultIntersectPoint1(0).x, shdl1.GetResultIntersectPoint1(0).y, shdl1.GetResultIntersectPoint2(0).x, shdl1.GetResultIntersectPoint2(0).y);
                //}
                //if (cmbxItem == "SHDLBottom" && (shdl2.GetResultNumLines() == 1))
                //{
                //    g.DrawLine(pen, shdl2.GetResultIntersectPoint1(0).x, shdl2.GetResultIntersectPoint1(0).y, shdl2.GetResultIntersectPoint2(0).x, shdl2.GetResultIntersectPoint2(0).y);
                //}
                //if (cmbxItem == "SHDLLeft" && (shdl3.GetResultNumLines() == 1))
                //{
                //    g.DrawLine(pen, shdl3.GetResultIntersectPoint1(0).x, shdl3.GetResultIntersectPoint1(0).y, shdl3.GetResultIntersectPoint2(0).x, shdl3.GetResultIntersectPoint2(0).y);
                //}
                //if (cmbxItem == "SHDLRight" && (shdl4.GetResultNumLines() == 1))
                //{
                //    g.DrawLine(pen, shdl4.GetResultIntersectPoint1(0).x, shdl4.GetResultIntersectPoint1(0).y, shdl4.GetResultIntersectPoint2(0).x, shdl4.GetResultIntersectPoint2(0).y);
                //}

            }
            // Display processing time
            //g.DrawString(string.Format("First Execution Time = {0:0.00} ms (may include preprocessing)", m_ExecuteTime1), Font, brush, 0, 0);
            //g.DrawString(string.Format("Second Execution Time = {0:0.00} ms (normal execution)", m_ExecuteTime2), Font, brush, 0, Font.Height);
        }

        private void btn_test_Click(object sender, EventArgs e)
        {
            draw_roi = false;
            Bitmap bitmap = new Bitmap(openFileDialog1.FileName);

            //img_disp.Image = imgInput.ToBitmap();
            //CProImage image_cpro = new CProImage();
            //folder check
            //folderBrowserDialog3.ShowDialog();
            //string foldername = folderBrowserDialog3.SelectedPath;
            //DirectoryInfo folder;
            //folder = new DirectoryInfo(foldername);
            //FileInfo[] images;
            //images = folder.GetFiles();
            //for (int i = 0; i < images.Length; i++)
            //{
            //    name = images[i].FullName;
            //    imgInput = new Mat(name, ImreadModes.Grayscale);
            //   // imgInput = new Mat(openFileDialog1.FileName, ImreadModes.Grayscale);
            //    img_disp.Image = imgInput.ToBitmap();
            //    imgInput.CopyTo(img1);
            //    Image.Set(imgInput.Width, imgInput.Height, CProData.FormatEnum.FormatUByte, imgInput.Data, true);

            //    if (Image.Valid)
            //        process_image_sapera(Image, true);
            //    update_display();
            //   // TO PASSTHE VALUES TO FORM3
            //   m = pass_fail;
            //}
            //one by one check
            imgInput = new Mat(openFileDialog1.FileName, ImreadModes.Grayscale);
            img_disp.Image = imgInput.ToBitmap();
            imgInput.CopyTo(img1);
            Image.Set(imgInput.Width, imgInput.Height, CProData.FormatEnum.FormatUByte, imgInput.Data, true);

            if (Image.Valid)
                process_image_sapera(Image, true);
            update_display();
            // TO PASSTHE VALUES TO FORM3
            m = pass_fail;


        }

        private void button2_Click(object sender, EventArgs e)
        {
            iomodule.WriteSingleCoil(1280, false);
        }

        private void FitImg_Click(object sender, EventArgs e)
        {
            img_disp.Width = panel1.Width - 20;
            img_disp.Height = panel1.Height - 20;
            img_disp.SizeMode = PictureBoxSizeMode.StretchImage;
        }

        private void checkedListBox1_SelectedIndexChanged(object sender, EventArgs e)
        {
           enableHistEqlze[cmbBx_model.SelectedIndex] = checkedListBox1.GetItemChecked(0);
            enableThreshold[cmbBx_model.SelectedIndex] = checkedListBox1.GetItemChecked(6);
           enableEqualize[cmbBx_model.SelectedIndex] = checkedListBox1.GetItemChecked(1);
            SourceLow.Value = (decimal)srcLow[cmbBx_model.SelectedIndex];
            SourceHigh.Value = srcHigh[cmbBx_model.SelectedIndex]==0?50:(decimal)srcHigh[cmbBx_model.SelectedIndex];
            DestLow.Value = (decimal)dstLow[cmbBx_model.SelectedIndex];
            DestHigh.Value = dstHigh[cmbBx_model.SelectedIndex]==0?255:(decimal)dstHigh[cmbBx_model.SelectedIndex];
            EdgeThreshold.Value = edgeThresh[cmbBx_model.SelectedIndex]==0?128:edgeThresh[cmbBx_model.SelectedIndex];
            thlo.Value = (decimal)threshlo[cmbBx_model.SelectedIndex];
            thhi.Value = (decimal)threshhi[cmbBx_model.SelectedIndex];
            thinv.Checked = threshinv[cmbBx_model.SelectedIndex];
           
            enableEdgeDetect[cmbBx_model.SelectedIndex] = checkedListBox1.GetItemChecked(2);
           enableSearchEdge[cmbBx_model.SelectedIndex] = checkedListBox1.GetItemChecked(3);
            if (enableSearchEdge[cmbBx_model.SelectedIndex])
            {
                if (!edgeSearches.ContainsKey(cmbBx_model.SelectedItem.ToString()))
                {
                    edgeSearches[cmbBx_model.SelectedItem.ToString()] = new CProSearchEdge();
                }
            }
           enableSearchArea[cmbBx_model.SelectedIndex] = checkedListBox1.GetItemChecked(4);
            if(enableSearchArea[cmbBx_model.SelectedIndex])
            {
                if (!areaSearches.ContainsKey(cmbBx_model.SelectedItem.ToString()))
                {
                    areaSearches[cmbBx_model.SelectedItem.ToString()] = new CProSearchArea();
                }
            }
            enableHoughCircle[cmbBx_model.SelectedIndex] = checkedListBox1.GetItemChecked(5);
            if(enableHoughCircle[cmbBx_model.SelectedIndex])
            {
                if (!HoughCircles.ContainsKey(cmbBx_model.SelectedItem.ToString()))
                {
                    HoughCircles[cmbBx_model.SelectedItem.ToString()] = new CProHoughCircle();
                }
            }
            update_display();
        }

        private void numericUpDown1_ValueChanged(object sender, EventArgs e)
        {
            if (enableSearchEdge[cmbBx_model.SelectedIndex])
            {
                edgeSearches[cmbBx_model.SelectedItem.ToString()].MinScore = (float)numericUpDown1.Value;
               // minscore[cmbBx_model.SelectedIndex] =(float) numericUpDown1.Value;
            }
            else if (enableSearchArea[cmbBx_model.SelectedIndex])
            {
                areaSearches[cmbBx_model.SelectedItem.ToString()].MinScore = (float)numericUpDown1.Value;
            }
            update_display();
        }

        private void SourceLow_ValueChanged(object sender, EventArgs e)
        {
            srcLow[cmbBx_model.SelectedIndex] = (float)SourceLow.Value;
            update_display();
        }

        private void SourceHigh_ValueChanged(object sender, EventArgs e)
        {
            srcHigh[cmbBx_model.SelectedIndex] = (float)SourceHigh.Value;
            update_display();
        }

        private void DestLow_ValueChanged(object sender, EventArgs e)
        {
            dstLow[cmbBx_model.SelectedIndex] = (float)DestLow.Value;
            update_display();
        }

        private void DestHigh_ValueChanged(object sender, EventArgs e)
        {
            dstHigh[cmbBx_model.SelectedIndex] = (float)DestHigh.Value;
            update_display();
        }

        private void EdgeThreshold_ValueChanged(object sender, EventArgs e)
        {
            edgeThresh[cmbBx_model.SelectedIndex] = (int)EdgeThreshold.Value;
            update_display();
        }

        private void checkBox1_CheckedChanged(object sender, EventArgs e)
        {
            drawres = checkBox1.Checked;
            update_display();
        }

        private void timer2_Tick(object sender, EventArgs e)
        {
            try
            {
                bool[] data = iomodule.ReadDiscreteInputs(1025, 4);

                for (int i = 0; i < data.Length; i++)
                {
                    Console.WriteLine("output is: " + data[i].ToString());
                }

                trig1 = trig2;
                trig2 = data[1];

                if (trig1 && !trig2)
                {
                    // Thread.Sleep(trig_delay-200);
                    iomodule.WriteSingleCoil(1284, false);
                    iomodule.WriteSingleCoil(1283, false);
                    ////// iomodule.WriteSingleCoil(1281, false);
                    Thread.Sleep(200);
                    if (sftTrig)
                        usbcam.ExecuteSoftwareTrigger();
                }
            }
            catch (Exception ex)
            {
                tlStrpStsLbl1.Image = global::Brakes_India.Properties.Resources.error;
                tlStrpStsLbl1.Text = "IO Module Error:" + ex.Message;

            }
        }

        private void img_disp_Click(object sender, EventArgs e)
        {

        }

        private void lbl_pass_fail_Click(object sender, EventArgs e)
        {

        }

        private void textBox5_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox7_TextChanged(object sender, EventArgs e)
        {

        }

        private void groupBox3_Enter(object sender, EventArgs e)
        {

        }

        private void textBox7_TextChanged_1(object sender, EventArgs e)
        {

        }

        private void label17_Click(object sender, EventArgs e)
        {

        }

        private void label21_Click(object sender, EventArgs e)
        {

        }

        private void label20_Click(object sender, EventArgs e)
        {

        }

        private void textBox3_TextChanged(object sender, EventArgs e)
        {

        }

        private void label5_Click(object sender, EventArgs e)
        {

        }

        private void thlo_ValueChanged(object sender, EventArgs e)
        {
            threshlo[cmbBx_model.SelectedIndex] = (float)thlo.Value;
            update_display();
        }

        private void shdrwd_TextChanged(object sender, EventArgs e)
        {

        }

        private void thhi_ValueChanged(object sender, EventArgs e)
        {
            threshhi[cmbBx_model.SelectedIndex] = (float)thhi.Value;
            update_display();
        }

        private void thinv_CheckedChanged(object sender, EventArgs e)
        {
            threshinv[cmbBx_model.SelectedIndex] = thinv.Checked;
            update_display();
        }

        private void btnIspectOnce_Click(object sender, EventArgs e)
        {
            draw = false;
            draw_roi = false;
            usbcam.Parameters[PLCamera.TriggerSource].TrySetValue(PLCamera.TriggerSource.Software);
            usbcam.Parameters[PLCamera.TriggerMode].TrySetValue(PLCamera.TriggerMode.On);

            try
            {
                // Starts the grabbing of one image.
                usbcam.Parameters[PLCamera.AcquisitionMode].SetValue(PLCamera.AcquisitionMode.SingleFrame);
                btnrun = true;
                ckBox_SH.Enabled = false;
                ckBox_ST.Enabled = false;
                ckBox_ET.Enabled = false;
                btnStop.Enabled = true;
                btnStart.Enabled = false;
                btnIspectOnce.Enabled = false;
                //iomodule.WriteSingleCoil(1283, true);
                Thread.Sleep(200);
                usbcam.StreamGrabber.Start(1, GrabStrategy.OneByOne, GrabLoop.ProvidedByStreamGrabber);
                if(sftTrig)
                    usbcam.ExecuteSoftwareTrigger();
              // to pass the array values to form3
                m = pass_fail;
                Form3 st = new Form3();
                st.Close();
            }
            catch (Exception exception)
            {
               throw(exception);
            }
        }

        public void btnStart_Click(object sender, EventArgs e)
        {
            draw = false;
            draw_roi = false;
            usbcam.Parameters[PLCamera.TriggerSource].TrySetValue(PLCamera.TriggerSource.Software);
            usbcam.Parameters[PLCamera.TriggerMode].TrySetValue(PLCamera.TriggerMode.On);

            try
            {
                // Starts the grabbing of one image.
                usbcam.Parameters[PLCamera.AcquisitionMode].SetValue(PLCamera.AcquisitionMode.Continuous);
                btnrun = true;
            ckBox_SH.Enabled = false;
            ckBox_ST.Enabled = false;
            ckBox_ET.Enabled = false;
            btnStop.Enabled = true;
            btnStart.Enabled = false;
                btnIspectOnce.Enabled = false;
                usbcam.StreamGrabber.Start(GrabStrategy.OneByOne, GrabLoop.ProvidedByStreamGrabber);
                // to pass the values to form3
                m = pass_fail;
                Form3 st = new Form3();
                st.Close();

            }
            catch (Exception exception)
            {
                throw (exception);
            }

        }


        private void cmdReset_Click(object sender, EventArgs e)
        {
            pass = 0;
            fail = 0;
            tot = 0;
            lblFailed.Text = fail.ToString();
            lblTotal.Text = tot.ToString();
            lblPassed.Text = pass.ToString();
            lbl_pass_fail.Text = "Pass";
            lbl_pass_fail.BackColor = Color.LightGreen;

        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            tlStrpStsLbl2.Text = DateTime.Now.ToString();
        }

        private void cmbBx_model_SelectedIndexChanged(object sender, EventArgs e)
        {
            //if(btnStart.Enabled)
            //    usbcam.Parameters[PLCamera.ExposureTime].TrySetValue(cam_expo);
            update_parameters();
            update_display();
        }

        private void update_parameters()
        {

            srcHigh[cmbBx_model.SelectedIndex] = srcHigh[cmbBx_model.SelectedIndex] == 0 ? 50 : srcHigh[cmbBx_model.SelectedIndex];
            dstHigh[cmbBx_model.SelectedIndex] = dstHigh[cmbBx_model.SelectedIndex] == 0 ? 50 : dstHigh[cmbBx_model.SelectedIndex];
            edgeThresh[cmbBx_model.SelectedIndex] = edgeThresh[cmbBx_model.SelectedIndex] == 0 ? 128 : edgeThresh[cmbBx_model.SelectedIndex];
            SourceLow.Value = (decimal)srcLow[cmbBx_model.SelectedIndex];
            SourceHigh.Value = (decimal)srcHigh[cmbBx_model.SelectedIndex];
            DestHigh.Value = (decimal)dstHigh[cmbBx_model.SelectedIndex];
            DestLow.Value = (decimal)dstLow[cmbBx_model.SelectedIndex];
            thlo.Value = (decimal)threshlo[cmbBx_model.SelectedIndex];
            thhi.Value = (decimal)threshhi[cmbBx_model.SelectedIndex];
            thinv.Checked = threshinv[cmbBx_model.SelectedIndex];

            EdgeThreshold.Value = edgeThresh[cmbBx_model.SelectedIndex];
            checkedListBox1.SetItemChecked(0, enableHistEqlze[cmbBx_model.SelectedIndex]);
            checkedListBox1.SetItemChecked(1, enableEqualize[cmbBx_model.SelectedIndex]);
            checkedListBox1.SetItemChecked(2, enableEdgeDetect[cmbBx_model.SelectedIndex]);
            checkedListBox1.SetItemChecked(3, enableSearchEdge[cmbBx_model.SelectedIndex]);
            checkedListBox1.SetItemChecked(4, enableSearchArea[cmbBx_model.SelectedIndex]);
            checkedListBox1.SetItemChecked(5, enableHoughCircle[cmbBx_model.SelectedIndex]);
            checkedListBox1.SetItemChecked(6, enableThreshold[cmbBx_model.SelectedIndex]);
            if (enableSearchEdge[cmbBx_model.SelectedIndex])
            {
                numericUpDown1.Value = (decimal)edgeSearches[cmbBx_model.SelectedItem.ToString()].MinScore;
            }
            else if (enableSearchArea[cmbBx_model.SelectedIndex])
           {
                numericUpDown1.Value = (decimal)areaSearches[cmbBx_model.SelectedItem.ToString()].MinScore;
            }
        }

        private void btn_train_minus_Click(object sender, EventArgs e)
        {
            draw_roi = true;
            draw = false;
            imgInput = new Mat(openFileDialog1.FileName, ImreadModes.Grayscale);
            Bitmap bitmap = BitmapConverter.ToBitmap(imgInput);
            img_disp.Image = bitmap;
            img_disp.SizeMode = PictureBoxSizeMode.AutoSize;
            //Mat opimg = new Mat(openFileDialog1.FileName, ImreadModes.GrayScale);
            //CProImage image_cpro = new CProImage();
            Image.Set(imgInput.Width, imgInput.Height, CProData.FormatEnum.FormatUByte, imgInput.Data, false);
            temp_no = 1;
        }

        private void btn_select_clr_Click(object sender, EventArgs e)
        {
            draw = false;
            imgInput = new Mat(openFileDialog1.FileName, ImreadModes.Color);
            Mat img_lab = new Mat();
            Cv2.CvtColor(imgInput, img_lab, ColorConversionCodes.BGR2HSV);
            Mat cimg = new Mat();
            Mat gimg = new Mat();
            //Cv2.MedianBlur(imgInput, cimg, 3);
            imgInput.CopyTo(cimg);
            Cv2.CvtColor(cimg, gimg,ColorConversionCodes.BGR2GRAY);
            CircleSegment[] clrs;
            clrs = Cv2.HoughCircles(gimg, HoughMethods.Gradient, 1, 100,100,100,100,180);
            Scalar scr = new Scalar(Bm, Gm, Rm);
            Scalar scr2 = new Scalar(Bx, Gx, Rx);
            Scalar scr3 = new Scalar(0, Gm, Rm);
            Scalar scr4 = new Scalar(15, Gx, Rx);
            Mat ot = new Mat();
            Mat ot2 = new Mat();
            Cv2.InRange(img_lab, scr, scr2, ot);
            Cv2.InRange(img_lab, scr3, scr4, ot2);
            Cv2.BitwiseXor(ot,ot2,ot);
            OpenCvSharp.Size sz = new OpenCvSharp.Size(3, 3);
            Mat ele = new Mat(sz, MatType.CV_8UC1, 1);
            Cv2.MorphologyEx(ot, ot, MorphTypes.Open, ele);
            //Cv2.ImShow("ot",ot);
            //Cv2.WaitKey(0);
            for (int i = 0; i < clrs.Length; i++)
            {
                Scalar clr = new Scalar(255,0,0);
                Mat temp = new Mat(ot.Rows, ot.Cols, MatType.CV_8UC1, 0);
                Cv2.Circle(temp, (int)clrs[i].Center.X, (int)clrs[i].Center.Y, (int)clrs[i].Radius, 255,-1);
                ot.CopyTo(temp, temp);
                if (Cv2.CountNonZero(temp) > 100)
                {
                    Cv2.Circle(cimg, (int)clrs[i].Center.X, (int)clrs[i].Center.Y,3, clr, 3);
                }
                Cv2.Circle(cimg, (int)clrs[i].Center.X, (int)clrs[i].Center.Y, (int)clrs[i].Radius, clr, 3);
            }
            img_disp.Image = ot.ToBitmap();
           // img_disp.Image = ot.ToBitmap();
            img_disp.SizeMode = PictureBoxSizeMode.AutoSize;
           
        }
    }
    public class ControlWriter : TextWriter
    {
        private Control textbox;
        public ControlWriter(Control textbox)
        {
            this.textbox = textbox;
        }

        public override void Write(char value)
        {
            textbox.Text += value;
        }

        public override void Write(string value)
        {
            textbox.Text += value;
        }

        public override Encoding Encoding
        {
            get { return Encoding.ASCII; }
        }
    }

}
