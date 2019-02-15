using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using OpenCvSharp;
using OpenCvSharp.Extensions;
using OpenCvSharp.UserInterface;
using OpenCvSharp.Blob;
using System.Drawing.Imaging;
using System.IO;
using Basler.Pylon;
using EasyModbus;
using System.Threading;



namespace top_bottom
{
    public partial class Exide_inspection : Form
    {
        public Exide_inspection()
        {
            InitializeComponent();
        }
        private static readonly log4net.ILog log =
            log4net.LogManager.GetLogger(System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);
        int i = 0;
        PixelDataConverter converter = new PixelDataConverter();
        bool draw = false;
        Mat temp1 = new Mat();
        Rectangle rect;
        OpenCvSharp.Point StartLocation;
        OpenCvSharp.Point EndLcation;
        bool IsMouseDown = false;
        Mat imgInput= new Mat();
        Mat templ1 = new Mat();
        bool btnrun = false;
        bool sftTrig = true;
        bool extTrig = false;
        bool trig1 = false, trig2 = false;
        bool sl_loaded = false;
        bool img_save = false;
        Camera usbcam;
        ModbusClient iomodule;
        int temp_no = 0;
        Mat cathode_temp = new Mat();
        Mat anode_temp = new Mat();
        int Cathode_terminal;
        int Anode_terminal;
       OpenCvSharp.Rect tc;
        OpenCvSharp.Rect ta;
        OpenCvSharp.Rect tclr;
        int Xc, Yc, Xa, Ya, Wc, Wa, Hc, Ha;
        int Xclr, Yclr ,Wclr,Hclr;
       
        int Hm = 5, Sm = 30, Vm = 40, Hx = 358, Sx = 81, Vx = 50;


        private void basler_gigi_load(object sender, EventArgs e)
        {
            btn_train1.Enabled = false;
            btn_train2.Enabled = false;
            btn_test.Enabled = false;
            try
            {
                List<ICameraInfo> allCameras = CameraFinder.Enumerate();
                if (allCameras.Count == 0)
                {
                    //  tlStrpPrgsBar1.Value = 100;
                    throw new Exception("No devices found.");
                }
                usbcam = new Camera();
               // iomodule = new ModbusClient("COM4");
               // iomodule.Parity = System.IO.Ports.Parity.Even;
              //  iomodule.Connect();
                timer2.Enabled = false;
                usbcam.Open();
                string oldPixelFormat = usbcam.Parameters[PLCamera.PixelFormat].GetValue(); // Remember the current pixel format.
                Console.WriteLine("Old PixelFormat  : {0} ({1})", usbcam.Parameters[PLCamera.PixelFormat].GetValue(), oldPixelFormat);

                Console.WriteLine("count  : {0}", usbcam.Parameters[PLCamera.AcquisitionMode].GetValue());
                if (!usbcam.Parameters[PLCamera.PixelFormat].TrySetValue(PLCamera.PixelFormat.BayerRG8))
                {
                    /* Feature is not available. */
                    throw new Exception("Device doesn't support the Mono8 pixel format.");
                }

                usbcam.Parameters[PLCamera.TriggerMode].TrySetValue(PLCamera.TriggerMode.Off);
                // usbcam.Parameters[PLCamera.AcquisitionMode].TrySetValue(PLCamera.AcquisitionMode.Continuous);
                if (extTrig)
                {
                    usbcam.Parameters[PLCamera.TriggerSource].TrySetValue(PLCamera.TriggerSource.Line1);
                    usbcam.Parameters[PLCamera.TriggerMode].TrySetValue(PLCamera.TriggerMode.On);
                }
                usbcam.StreamGrabber.ImageGrabbed += OnImageGrabbed;
                usbcam.StreamGrabber.GrabStopped += OnGrabStopped;
                txtBox.Text += "Camera connected\r\n";



            }
            catch (Exception ex)
            {
                tlStrpPrgsBar1.Value = 100;
                tlStrpStsLbl1.Image = global::top_bottom.Properties.Resources.error;
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
                //ckBox_SH.Enabled = false;
                //ckBox_ST.Enabled = false;
                //ckBox_ET.Enabled = false;
                btnIspectOnce.Enabled = false;
                txtBox.Text += "\nPress enter to exit.\r\n";
            }

        }
        private void OnImageGrabbed(Object sender, ImageGrabbedEventArgs e)
        {
            draw = false;
            //draw_roi = false;
            if (InvokeRequired)
            {
                // If called from a different thread, we must use the Invoke method to marshal the call to the proper GUI thread.
                // The grab result will be disposed after the event call. Clone the event arguments for marshaling to the GUI thread.
                BeginInvoke(new EventHandler<ImageGrabbedEventArgs>(OnImageGrabbed), sender, e.Clone());
                return;
            }
            Thread.Sleep(200);
           // iomodule.WriteSingleCoil(1283, false);
            tlStrpPrgsBar1.Value = 20;
            Mat img, grayimg = new Mat();
            if (btnrun == true)
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
                    if (img_save)
                        Cv2.ImWrite(folderBrowserDialog1.SelectedPath + "\\" + i.ToString() + ".bmp", img);
                    Bitmap bitmapOld = img_disp.Image as Bitmap;
                    img_disp.Image = bitmap;
                    if (bitmapOld != null)
                    {
                        // Dispose the bitmap.
                        bitmapOld.Dispose();
                    }
                    //img.CopyTo(img1);
                    if (btn_right.Enabled == false)
                    {


                        Bitmap img2 = img.ToBitmap();
                        img2.RotateFlip(RotateFlipType.Rotate180FlipNone);

                        img = BitmapConverter.ToMat(img2);
                        // Cv2.ImShow("rotate", image);

                        process_image(img);
                    }
                    else if (btn_right.Enabled == true)
                    {
                        process_image(img);
                    }
                   
                    Cv2.CvtColor(img, grayimg, ColorConversionCodes.BGR2GRAY);
                    //Image.Set(bitmap.Width, bitmap.Height, CProData.FormatEnum.FormatUByte, grayimg.Data, true);
                    //process_image_sapera(Image);
                }
                else if (!res.GrabSucceeded)
                {
                    SetText(String.Format("Frame {0} wasn't grabbed successfully.  Error code = {1}\r\n", i + 1, res.ErrorCode));
                }
                ++i;
            }
            tlStrpPrgsBar1.Value = 100;
            tlStrpStsLbl1.Text = String.Format("Frame {0} grabbed.", i + 1);
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
            //ckBox_SH.Enabled = true;
            ckBox_ST.Enabled = true;
            btnStop.Enabled = true;
            btnStart.Enabled = false;
            btnIspectOnce.Enabled = false;
        }
        private void process_image(Mat imgIn, bool offline = false)
        {
            tlStrpPrgsBar1.Value = 0;
            Mat img = new Mat();
            imgIn.CopyTo(imgInput);
            Cv2.CvtColor(imgIn, img, ColorConversionCodes.BGR2GRAY);
            Mat cimg = new Mat();
            Mat tempc = new Mat();
            Mat tempa = new Mat();
            Mat gimg = new Mat();
            Mat tempclr = new Mat();
            

            if (cmBox_clr.Checked)
            {
             
                tclr = new Rect(Xclr, Yclr, Wclr, Hclr);
                tempclr = new Mat(imgInput, tclr);

                tempclr.CopyTo(cimg);
                Mat imglab = new Mat();
                Cv2.CvtColor(cimg, imglab, ColorConversionCodes.BGR2HSV);
            //    Cv2.CvtColor(cimg, gimg, ColorConversionCodes.BGR2GRAY);
                Scalar scr1 = new Scalar(Hm, Sm, Vm);
                Scalar scr2 = new Scalar(Hx, Sx, Vx);
                Mat ot = new Mat();
                // Mat ot2 = new Mat();
                Cv2.InRange(imglab, scr1, scr2, ot);
                OpenCvSharp.Size sz = new OpenCvSharp.Size(3, 3);
                Mat ele = new Mat(sz, MatType.CV_8UC1, 1);
                Cv2.MorphologyEx(ot, ot, MorphTypes.Dilate, ele);
                Cv2.ImShow("ot", ot);
                int countpixel = 0;


                for (int x = 0; x < ot.Rows; x++)
                {
                    for (int y = 0; y < ot.Cols; y++)
                    {
                        if (ot.At<char>(x, y) == 255)
                        {
                            countpixel++; tlStrpPrgsBar1.Value = 50;
                        }
                    }
                }
                clr_p.Text = countpixel.ToString();
                Cv2.Rectangle(imgInput, tclr, Scalar.Green,3);
            }
             //  if (temp_no==1)
                {
                 tc = new Rect(Xc, Yc, Wc, Hc);
                tempc = new Mat(img, tc);
             //   Cv2.NamedWindow("tempc1", WindowMode.Normal);
               // Cv2.ImShow("tempc1", tempc);
               // cal_grad(tempc, tempc);
             //   Cv2.NamedWindow("tempc2", WindowMode.AutoSize);
             //   Cv2.ImShow("tempc2", tempc);



            }
             //  else if (temp_no==2)
                {
                 ta = new Rect(Xa, Ya, Wa, Ha);
                tempa = new Mat(img, ta);
            //    Cv2.NamedWindow("tempa1", WindowMode.Normal);
              //  Cv2.ImShow("tempa1", tempa);
            }
            
               

         //    if (temp_no==1)  
            {
                            {
                    Cathode_terminal = 0;
        
                
                   OpenCvSharp.Size ksize = new OpenCvSharp.Size(3, 3);
                   Cv2.GaussianBlur(tempc, tempc, ksize, 3);
                    Cv2.MedianBlur(tempc,tempc,3);

                 
                  //  Cv2.NamedWindow("tempc", WindowMode.AutoSize);
                  //  Cv2.ImShow("tempc", tempc);
                       Cv2.Threshold(tempc, tempc, 230, 255, ThresholdTypes.Binary);
                 //  Cv2.NamedWindow("tempc", WindowMode.Normal);
                //    Cv2.ImShow("tempc", tempc);
                   

                    OpenCvSharp.Point[][] store;

                    
                    HierarchyIndex[] hey;
                    Cv2.FindContours(tempc, out store, out hey, RetrievalModes.Tree, ContourApproximationModes.ApproxSimple);
                    OpenCvSharp.Rect rect1;
                    

                    for (int i = 0; i < store.Length; i++)
                    {
                        Scalar pen2 = new Scalar(0, 255, 0);
                        if (Cv2.ContourArea(store[i], false) > 1600 && Cv2.ContourArea(store[i], false) < 6000)
                        {
                           // Cv2.DrawContours(imgInput, store, i, pen2, 2);
                            rect1 = Cv2.BoundingRect(store[i]);
                           int X= rect1.X + Xc;
                            int Y = rect1.Y + Yc;
                            Rect rect2 = new Rect(X, Y, rect1.Width, rect1.Height);
                            // Cv2.Rectangle(imgInput, rect1, Scalar.Orange, 2);
                            Cv2.Rectangle(imgInput, rect2, Scalar.Blue, 3);
                            Cathode_terminal++;
                        }

                    }


                    Cathode_Count.Text = Cathode_terminal.ToString();
                    Cv2.Rectangle(imgInput,tc, Scalar.Green,2);
                }
            //  if (temp_no == 2)
                {
                    Anode_terminal = 0;
                   
                    OpenCvSharp.Size ksize = new OpenCvSharp.Size(3, 3);
                    Cv2.GaussianBlur(tempa, tempa, ksize, 3);
                    Cv2.MedianBlur(tempa, tempa, 3);
                    Cv2.EqualizeHist(tempa,tempa);
                //    Cv2.Invert(tempa, tempa);
                   // Cv2.InvertAffineTransform(tempa,tempa);
                   
               
                 
                   Mat element = Cv2.GetStructuringElement(MorphShapes.Rect, ksize);
                    Cv2.Dilate(tempa,tempa,element);
                    // Cv2.Erode(tempa, tempa, element);
                    // Cv2.Canny(tempa, tempa);
                    Cv2.Threshold(tempa, tempa, 246, 255, ThresholdTypes.Binary);
                    Cv2.NamedWindow("tempa2", WindowMode.Normal);
                    Cv2.ImShow("tempa2", tempa);
                    cal_grad(tempa,tempa);
                    Cv2.NamedWindow("tempaa", WindowMode.Normal);
                    Cv2.ImShow("tempaa",tempa);
                   // Cv2.EqualizeHist(tempa, tempa);
                   
                    // Cv2.Canny(timg, timg, 1, 255);
                 
                    // Mat store = new Mat();
                    OpenCvSharp.Point[][] store;

                    // Mat hey = new Mat();
                    HierarchyIndex[] hey;
                    Cv2.FindContours(tempa, out store, out hey, RetrievalModes.Tree, ContourApproximationModes.ApproxSimple);
                    //  int n = store.Length;
                    OpenCvSharp.Rect rect1;


                    for (int i = 0; i < store.Length; i++)
                    {
                        Scalar pen2 = new Scalar(0, 255, 0);
                        if (Cv2.ContourArea(store[i], false) > 1800 && Cv2.ContourArea(store[i], false) < 3500)
                        {
                            //Cv2.DrawContours(imgInput, store, i, pen2, 2);
                            rect1 = Cv2.BoundingRect(store[i]);
                            int X = rect1.X + Xa;
                            int Y = rect1.Y + Ya;
                            Rect rect2 = new Rect(X, Y, rect1.Width, rect1.Height);
                            Cv2.Rectangle(imgInput, rect2, Scalar.Blue, 2);
                            Anode_terminal++;
                        }

                    }


                    Anode_Count.Text = Anode_terminal.ToString();
                  
                    Cv2.Rectangle(imgInput, ta,Scalar.Green,2);
                }
                // int countpixel = 0;
            }


             int cat_inn=(int)Cat_in.Value;
            int An_inn = (int)An_in.Value;
          

            if (cat_inn==Cathode_terminal &&An_inn==Anode_terminal)
            {
                pass_fail_result.Text = "PASS";
                pass_fail_result.BackColor = Color.Green;
            }
            else
            {
                pass_fail_result.Text = "FAIL";
                pass_fail_result.BackColor = Color.Red;
              //  iomodule.WriteSingleCoil(1280, true);
            }
        


            tlStrpPrgsBar1.Value = 70;

            img_disp.Image = imgInput.ToBitmap();

            tlStrpPrgsBar1.Value = 100;
        }
        private void cal_grad(Mat iimg, Mat oimg)
        {
            Mat img = new Mat();
            iimg.CopyTo(img);
            Mat grad_x = new Mat();
            Mat grad_y = new Mat();
            double[] g = new double[] { 0.1353, 0.6065, 0, 0.6065, 0.1353 };
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
        private void basler_gige_closing(object sender, FormClosingEventArgs e)
        {
            //tcpclnt.Close();
            /* Release the buffer. */

            if (usbcam != null)
            {
                usbcam.Close();
                usbcam.Dispose();
            }

            txtBox.Text += "\nPress enter to exit.\r\n";
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
            if (draw)
            {
                if (rect != null)
                {
                    e.Graphics.DrawRectangle(Pens.LightYellow, GetRectangle());
                }
            }
        }

        private void img_disp_MouseDown(object sender, MouseEventArgs e)
        {
            if (draw)
            {
                IsMouseDown = true;
                StartLocation = new OpenCvSharp.Point(e.X, e.Y);
            }
        }

        private void img_disp_MouseMove(object sender, MouseEventArgs e)
        {
            if (draw)
            {
                if (IsMouseDown == true)
                {
                    EndLcation = new OpenCvSharp.Point(e.X, e.Y);
                    img_disp.Invalidate();
                }
            }
        }

        private void img_disp_MouseUp(object sender, MouseEventArgs e)
        {
            if (draw)
            {
                if (IsMouseDown == true)
                {
                    EndLcation = new OpenCvSharp.Point(e.X, e.Y);
                    IsMouseDown = false;
                    if (rect != null)
                    {
                        double scaleX = (double)imgInput.Width / img_disp.Size.Width;
                        double scaleY = (double)imgInput.Height / img_disp.Size.Height;
                        int X, Y, W, H;
                        X = (int)(rect.X * scaleX)+5;
                        Y = (int)(rect.Y * scaleY) + 10;
                        W = (int)(rect.Width * scaleX);
                        H = (int)(rect.Height * scaleY);
                        Rect t = new Rect(X,Y,W,H);
                        Mat temp = new Mat(imgInput, t);
                        if(temp_no==1)
                        {
                            // if(Cathode_btn.Checked)
                            // cathode = t;
                            Xc = X;
                            Yc = Y;
                            Wc = W;
                            Hc = H;
                              //  temp.CopyTo(cathode_temp);
                        }
                          else if(temp_no==2)
                        {
                            //anode = t;
                            Xa = X;
                            Ya = Y;
                            Wa = W;
                            Ha = H;
                           // temp.CopyTo(anode_temp);
                            
                        }
                        else if(temp_no==3)
                        {
                            Xclr = X;
                            Yclr = Y;
                            Wclr = W;
                            Hclr = H;
                        }
                      //  temp.CopyTo(templ1);
                        Bitmap bitmap = BitmapConverter.ToBitmap(temp);
                        pictureBox1.Image = bitmap;
                        pictureBox1.SizeMode = PictureBoxSizeMode.StretchImage;
                    }
                }
            }
        }

        private void btn_train_Click(object sender, EventArgs e)
        {
            draw = true;
            imgInput = new Mat(openFileDialog1.FileName, ImreadModes.GrayScale);
            Bitmap bitmap = BitmapConverter.ToBitmap(imgInput);
            img_disp.Image = bitmap;
            img_disp.SizeMode = PictureBoxSizeMode.StretchImage;
            btn_test.Enabled = true;
            temp_no = 1;
        }

        private void btn_browse_Click(object sender, EventArgs e)
        {
            openFileDialog1.ShowDialog();
            if (openFileDialog1.FileName == "")
            {
                MessageBox.Show("No file selected", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            txtImgFile.Text = openFileDialog1.FileName;
            btn_test.Enabled = true;
        }

        private void btn_test_Click(object sender, EventArgs e)
        {
            draw = false;
            if(btn_right.Enabled==false)
            {
                Mat image = new Mat(openFileDialog1.FileName,ImreadModes.Color);
                
                Bitmap img2 = image.ToBitmap();
                img2.RotateFlip(RotateFlipType.Rotate180FlipNone);

                image = BitmapConverter.ToMat(img2);
               // Cv2.ImShow("rotate", image);
                process_image(image, true);
            }
            else if(btn_right.Enabled==true)
            {
                process_image(new Mat(openFileDialog1.FileName, ImreadModes.Color), true);
            }
        }

        private void btn_load_Click(object sender, EventArgs e)
        {
            openFileDialog2.ShowDialog();
            if (openFileDialog2.FileName == "")
            {
                MessageBox.Show("No File selected", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            txt_save_file.Text = openFileDialog2.FileName;
            
            FileStorage fs = new FileStorage(txt_save_file.Text, FileStorage.Mode.Read);
            // cathode_temp = fs["cathode_temp"].ReadMat();
            // anode_temp = fs["anode_temp"].ReadMat();
            Xc = fs["Xc"].ReadInt();
            Yc = fs["Yc"].ReadInt();
            Wc = fs["Wc"].ReadInt();
            Hc = fs["Hc"].ReadInt();
            Xa = fs["Xa"].ReadInt();
            Ya = fs["Ya"].ReadInt();
            Wa = fs["Wa"].ReadInt();
            Ha = fs["Ha"].ReadInt();
            Xclr = fs["Xclr"].ReadInt();
            Yclr = fs["Yclr"].ReadInt();
            Wclr = fs["Wclr"].ReadInt();
            Hclr = fs["Hclr"].ReadInt();
            Hm = fs["Hs"].ReadInt();
            Sm = fs["Sm"].ReadInt();
            Vm = fs["Vm"].ReadInt();
            Hx = fs["Hx"].ReadInt();
            Sx = fs["Sx"].ReadInt();
            Vx = fs["Vx"].ReadInt();
            fs.Release();
        }

        private void btnSave_Click(object sender, EventArgs e)
        {
            draw = false;
            saveFileDialog1.ShowDialog();
            if (saveFileDialog1.FileName == "")
            {
                MessageBox.Show("No file saved", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            txt_save_file.Text = saveFileDialog1.FileName;
            FileStorage fs = new FileStorage(txt_save_file.Text, FileStorage.Mode.Write);
            //   fs.Write("cathode_temp", cathode_temp);
            // fs.Write("anode_temp", anode_temp);
            fs.Write("Xc", Xc);
            fs.Write("Yc", Yc);
            fs.Write("Wc", Wc);
            fs.Write("Hc", Hc);
            fs.Write("Xa", Xa);
            fs.Write("Ya", Ya);
            fs.Write("Wa", Wa);
            fs.Write("Ha", Ha);
            fs.Write("Hclr", Hclr);
            fs.Write("Wclr", Wclr);
            fs.Write("Xclr", Xclr);
            fs.Write("Yclr",Yclr);
            fs.Write("Hm", Hm);
            fs.Write("Sm", Sm);
            fs.Write("Vm", Vm);
            fs.Write("Hx", Hx);
            fs.Write("Sx", Sx);
            fs.Write("Vx", Vx);
            fs.Write("modified", DateTime.Now.ToString());
            fs.Release();
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
           // System.IO.Directory.CreateDirectory(folderBrowserDialog1.SelectedPath + "\\Pass");
           // System.IO.Directory.CreateDirectory(folderBrowserDialog1.SelectedPath + "\\Fail");
            img_save = true;
        }

        private void pictureBox2_Click(object sender, EventArgs e)
        {

        }

        private void img_disp_Click(object sender, EventArgs e)
        {

        }

        private void pixel_count_TextChanged(object sender, EventArgs e)
        {

        }

        private void btnStart_Click(object sender, EventArgs e)
        {
            draw = false;
           // draw_roi = false;
            usbcam.Parameters[PLCamera.TriggerSource].TrySetValue(PLCamera.TriggerSource.Software);
            usbcam.Parameters[PLCamera.TriggerMode].TrySetValue(PLCamera.TriggerMode.On);

            try
            {
                // Starts the grabbing of one image.
                usbcam.Parameters[PLCamera.AcquisitionMode].SetValue(PLCamera.AcquisitionMode.Continuous);
                btnrun = true;
                //ckBox_SH.Enabled = false;
                ckBox_ST.Enabled = false;
               // ckBox_ET.Enabled = false;
                btnStop.Enabled = true;
                btnStart.Enabled = false;
                btnIspectOnce.Enabled = false;
                usbcam.StreamGrabber.Start(GrabStrategy.OneByOne, GrabLoop.ProvidedByStreamGrabber);
                // to pass the values to form3
               
            }
            catch (Exception exception)
            {
                throw (exception);
            }
        }

        private void textBox6_TextChanged(object sender, EventArgs e)
        {

        }

        private void btnStop_Click(object sender, EventArgs e)
        {
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

        private void btnIspectOnce_Click(object sender, EventArgs e)
        {

            draw = false;
           // draw_roi = false;
           // //usbcam.Parameters[PLCamera.TriggerSource].TrySetValue(PLCamera.TriggerSource.Software);
           //// usbcam.Parameters[PLCamera.TriggerMode].TrySetValue(PLCamera.TriggerMode.On);
           // usbcam.Parameters[PLCamera.AcquisitionMode].SetValue(PLCamera.AcquisitionMode.Continuous);
          //  usbcam.StreamGrabber.Start(1, GrabStrategy.OneByOne, GrabLoop.ProvidedByStreamGrabber);

            try
            {
                // Starts the grabbing of one image.
                usbcam.Parameters[PLCamera.AcquisitionMode].SetValue(PLCamera.AcquisitionMode.SingleFrame);
                btnrun = true;
                //ckBox_SH.Enabled = false;
                ckBox_ST.Enabled = false;
                //ckBox_ET.Enabled = false;
                btnStop.Enabled = true;
                btnStart.Enabled = false;
                btnIspectOnce.Enabled = false;
              //  iomodule.WriteSingleCoil(1283, true);
                Thread.Sleep(200);
                usbcam.StreamGrabber.Start(1, GrabStrategy.OneByOne, GrabLoop.ProvidedByStreamGrabber);
                if (sftTrig)
                    usbcam.ExecuteSoftwareTrigger();
                // to pass the array values to form3
               
            }
            catch (Exception exception)
            {
                throw (exception);
            }
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            tlStrpStsLbl2.Text = DateTime.Now.ToString();
        }

        private void btn_clr_Click(object sender, EventArgs e)
        {
            draw = true;
            imgInput = new Mat(openFileDialog1.FileName, ImreadModes.GrayScale);
            Bitmap bitmap = BitmapConverter.ToBitmap(imgInput);
            img_disp.Image = bitmap;
            img_disp.SizeMode = PictureBoxSizeMode.StretchImage;
            btn_test.Enabled = true;
            temp_no = 3;
        }

        private void comb_Box_SelectedIndexChanged(object sender, EventArgs e)
        {

        }

        private void button1_Click_1(object sender, EventArgs e)
        {
            btn_left.Enabled = false;
            btn_right.Enabled = true;
            btn_test.Enabled = true;
            btn_train1.Enabled = true;
            btn_train2.Enabled = true;
            btnStart.Enabled = true;
            btnIspectOnce.Enabled = true;
        }

        private void btn_right_Click(object sender, EventArgs e)
        {
            btn_left.Enabled = true;
            btn_right.Enabled = false;
            btn_test.Enabled = true;
            btn_train1.Enabled = true;
            btn_train2.Enabled = true;
            btnStart.Enabled = true;
            btnIspectOnce.Enabled = true;
        }

        private void txtBox_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox1_TextChanged(object sender, EventArgs e)
        {

        }

        private void Cathode_btn_CheckedChanged(object sender, EventArgs e)
        {

        }

        private void btn_train2_Click(object sender, EventArgs e)
        {
            draw = true;
            imgInput = new Mat(openFileDialog1.FileName, ImreadModes.GrayScale);
            Bitmap bitmap = BitmapConverter.ToBitmap(imgInput);
            img_disp.Image = bitmap;
            img_disp.SizeMode = PictureBoxSizeMode.StretchImage;
            btn_test.Enabled = true;
            temp_no = 2;
        }

        private void textBox4_TextChanged(object sender, EventArgs e)
        {

        }

        private void lblPassed_Click(object sender, EventArgs e)
        {

        }

        private void textBox3_TextChanged(object sender, EventArgs e)
        {

        }

        private void timer2_Tick(object sender, EventArgs e)
        {

            try
            {
                bool[] data = iomodule.ReadDiscreteInputs(1024, 8);

                for (int i = 0; i < data.Length; i++)
                {
                    Console.WriteLine("output is: " + data[i].ToString());
                }
                trig1 = trig2;
                trig2 = data[0];
                if (trig1 && !trig2)
                {
                    //  Thread.Sleep(trig_delay - 200);
                    iomodule.WriteSingleCoil(1283, true);
                    Thread.Sleep(200);
                    if (sftTrig)
                        usbcam.ExecuteSoftwareTrigger();
                }
            }
            catch (Exception ex)
            {
                //   tlStrpStsLbl1.Image = global::seal_inspection.Properties.Resources.error;
                tlStrpStsLbl1.Text = "IO Module Error:" + ex.Message;
            }
        }
       
    }
}
