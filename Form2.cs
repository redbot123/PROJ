manjula
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;

namespace DotNetExample2
{
    public partial class Form1 : Form
    {
        IpeEngCtrlLib.Engine VisionSystem;
        IpeEngCtrlLib.I_ENG_ERROR iReturn;
        IpeEngCtrlLib.I_MODE iCurrMode;
        bool bContinuous = true;
        int iVSResize;
        int iHSResize;
        int iVSMove = 0;
        int iHSMove;
        double dBlobsExpected;
        double dThreshold;
        public Form1()
        {
            InitializeComponent();
            //Make sure to dispose the VisionSystem object in Dispose which is shown in this example
            VisionSystem = new IpeEngCtrlLib.Engine();
            iReturn = VisionSystem.EngInitialize();
            if (iReturn != IpeEngCtrlLib.I_ENG_ERROR.I_OK)
            {
                MessageBox.Show("Could not initialize VisionSystem object.", "FATAL ERROR", MessageBoxButtons.OK);
                Close();
            }

            // VisionSystem RunCompleted event handler definition
            VisionSystem.RunCompleted +=
                new IpeEngCtrlLib._IEngineEvents_RunCompletedEventHandler(VisionSystem_RunCompleted);
        }
        private void Form1_Load(object sender, EventArgs e)
        {
            iReturn = VisionSystem.InvLoad("simple4.ivs");//load investigation
            if (iReturn != IpeEngCtrlLib.I_ENG_ERROR.I_OK)// check for error
            {
                MessageBox.Show("Could not load investigation.", "FATAL ERROR", MessageBoxButtons.OK);

            }

            ImageWindow.ConnectEngine(VisionSystem.GetEngineObj());// connect display object to the VisionSystem object
            ImageWindow.ConnectImgWindow("imgA");// connect display to VisionSystem image window
            ImageWindow.SetZoom((-1));// set display zoom to stretch

            //----------------------Initialize Values---------------
            iVSMove = VS_Move.Value;
            iHSMove = HS_Move.Value;
            iVSResize = VS_Resize.Value;
            iHSResize = HS_Resize.Value;
            //---------------------Get last saved VisionSystem data
            iReturn = VisionSystem.VarGetDouble("BlobsExpected", out dBlobsExpected);
            lblExpectedBlobs.Text = "Expected Blobs = " + dBlobsExpected.ToString();
            HS_ExpectedBlobs.Value = (int)dBlobsExpected;
            iReturn = VisionSystem.VarGetDouble("Threshold", out dThreshold);
            lblThreshold.Text = "Threshold Value = " + dThreshold.ToString();
            HS_Threshold.Value = (int)dThreshold;
        }
        private void VisionSystem_RunCompleted()
        {
            double dTotal;
            double dPass;
            double dFail;
            double dBlobsFound;
            bool bInspPassResult;
            iReturn = VisionSystem.VarGetDouble("PartsInspectedCount",out dTotal);
            lblTotal.Text = dTotal.ToString();
            iReturn = VisionSystem.VarGetDouble("PassCount", out dPass);
            lblPassed.Text = dPass.ToString();
            iReturn = VisionSystem.VarGetDouble("FailCount",out dFail);
            lblFailed.Text = dFail.ToString();
            iReturn = VisionSystem.VarGetDouble("BlobsFound", out dBlobsFound);
            lblFound.Text = dBlobsFound.ToString();
            iReturn = VisionSystem.VarGetDouble("BlobsExpected", out dBlobsExpected);
            lblExpected.Text = dBlobsExpected.ToString();
            iReturn = VisionSystem.VarGetBool("Pass",out  bInspPassResult);
            if (bInspPassResult == true)  //  
            {
                lblResult.Text = "Passed";
                lblResult.BackColor = System.Drawing.Color.LightGreen;
            }
            else if (bInspPassResult == false) // 
            {
                lblResult.Text = "Failed";
                lblResult.BackColor = System.Drawing.Color.Tomato;
            }
        }
        public void Form1_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {

            if (bContinuous == true)//if Investigating Continuous, Halt Inspection
            {
                HaltInv();

            }
            ImageWindow.DisconnectImgWindow();//disconnect display window
        }        
  
        private void cmdClose_Click(object sender, EventArgs e)
        {
            ImageWindow.DisconnectImgWindow();//disconnect display window
            Close();
        }

        private void cmdInspectOnce_Click(object sender, EventArgs e)
        {
            iReturn = VisionSystem.InvModeSet(IpeEngCtrlLib.I_MODE.I_EXE_MODE_ONCE);
        }

        private void cmdContinuous_Click(object sender, EventArgs e)
        {
            iReturn = VisionSystem.InvModeSet(IpeEngCtrlLib.I_MODE.I_EXE_MODE_CONT);
            bContinuous = true;
            cmdInspectOnce.Enabled = false;
            cmdContinuous.Enabled = false;
            cmdSave.Enabled = false;
        }

        private void cmdHalt_Click(object sender, EventArgs e)
        {
            HaltInv();
            bContinuous = false;
            cmdInspectOnce.Enabled = true;
            cmdContinuous.Enabled = true;
            cmdSave.Enabled = true;
        }

        private void cmdSave_Click(object sender, EventArgs e)
        {
            iReturn = VisionSystem.InvSave("..\\Programs\\simple4.ivs");
        }

        private void cmdReset_Click(object sender, EventArgs e)
        {
            iReturn = VisionSystem.VarSetDouble("PartsInspectedCount", 0);
            iReturn = VisionSystem.VarSetDouble("PassCount", 0);
            iReturn = VisionSystem.VarSetDouble("FailCount", 0);
            lblTotal.Text = "";
            lblPassed.Text = "";
            lblFailed.Text = "";
        }

        private void VS_Move_Scroll(object sender, ScrollEventArgs e)
        {
            if (VS_Move.Value > iVSMove)  // Move down 
            {
                iReturn = VisionSystem.RoiMove("RectA", 0, 1);
            }
            else if (VS_Move.Value < iVSMove) // Move up
            {
                iReturn = VisionSystem.RoiMove("RectA", 0, -1);
            }

            iVSMove = VS_Move.Value; // Save Vert. Scroll value to container
            ImageWindow.UpdateDisplay();
        }

        private void HS_Move_Scroll(object sender, ScrollEventArgs e)
        {
            if (HS_Move.Value > iHSMove) //Move Right
            {
                iReturn = VisionSystem.RoiMove("RectA", 1, 0);
            }
            else if (HS_Move.Value < iHSMove) //Move Left
            {
                iReturn = VisionSystem.RoiMove("RectA", -1, 0);
            }

            iHSMove = HS_Move.Value; //Save Vert. Scroll value to container
            ImageWindow.UpdateDisplay();
        }

        private void VS_Resize_Scroll(object sender, ScrollEventArgs e)
        {
            if (VS_Resize.Value > iVSResize)// Larger Vertically
            {
                iReturn = VisionSystem.RoiCoordMove("RectA", 0, 0, 1);
                iReturn = VisionSystem.RoiCoordMove("RectA", 1, 0, -1);
            }
            else if (VS_Resize.Value < iVSResize)// Smaller Vertically
            {
                iReturn = VisionSystem.RoiCoordMove("RectA", 0, 0, -1);
                iReturn = VisionSystem.RoiCoordMove("RectA", 1, 0, 1);
            }
            iVSResize = VS_Resize.Value; //Save Vert. Scroll value to container
            ImageWindow.UpdateDisplay();
        }

        private void HS_Resize_Scroll(object sender, ScrollEventArgs e)
        {
            if (HS_Resize.Value > iHSResize) // Larger Horizontally
            {
                iReturn = VisionSystem.RoiCoordMove("RectA", 0, -1, 0);
                iReturn = VisionSystem.RoiCoordMove("RectA", 1, 1, 0);
            }
            else if (HS_Resize.Value < iHSResize)// Smaller Horizontally
            {
                iReturn = VisionSystem.RoiCoordMove("RectA", 0, 1, 0);
                iReturn = VisionSystem.RoiCoordMove("RectA", 1, -1, 0);
            }
            iHSResize = HS_Resize.Value;//Save Vert. Scroll value to container
            ImageWindow.UpdateDisplay();
        }

        private void HS_Threshold_Scroll(object sender, ScrollEventArgs e)
        {
            iReturn = VisionSystem.VarSetDouble("Threshold", HS_Threshold.Value);//Set VisionSystem variable to Horz. Scroll value
            dThreshold = HS_Threshold.Value;
            lblThreshold.Text = "Threshold Value = " + dThreshold.ToString();// update label
        }

        private void HS_ExpectedBlobs_Scroll(object sender, ScrollEventArgs e)
        {
            iReturn = VisionSystem.VarSetDouble("BlobsExpected", HS_ExpectedBlobs.Value);//Set VisionSystem variable to Horz. Scroll value
            dBlobsExpected = HS_ExpectedBlobs.Value;
            lblExpectedBlobs.Text = "Expected Blobs = " + dBlobsExpected.ToString();// update label
        }

        private void HaltInv()
        {
            iReturn = VisionSystem.InvModeSet(IpeEngCtrlLib.I_MODE.I_EXE_MODE_HALT); // halt inspection after current inspection is finished
            while (iCurrMode != IpeEngCtrlLib.I_MODE.I_EXE_MODE_HALT) //stay in loop until Halt is complete
            {
                Application.DoEvents();
                iReturn = VisionSystem.InvModeGet(out iCurrMode);
            }
        }

    }
}