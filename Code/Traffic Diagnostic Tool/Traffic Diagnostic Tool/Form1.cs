using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace Traffic_Diagnostic_Tool
{
    public partial class Form1 : Form
    {
        int intersection = 0;
        string videofile = "";
        int heading = 0;
        int pvalue = 0;
        public object OpenFileDialog1 { get; private set; }

        public Form1()
        {
            InitializeComponent();
        }

        private void radioButton1_CheckedChanged(object sender, EventArgs e)
        {
            intersection = 0;
        }

        private void button2_Click(object sender, EventArgs e)
        {
            openFileDialog1.ShowDialog();
        }

        private void button3_Click(object sender, EventArgs e)
        {
            axWindowsMediaPlayer1.Ctlcontrols.play();
        }

        private void openFileDialog1_FileOk(object sender, CancelEventArgs e)
        {
            
            //textBox1.Text = axWindowsMediaPlayer1.URL;
            textBox1.Text = openFileDialog1.FileName;
            videofile = openFileDialog1.FileName;
        }

        private void button4_Click(object sender, EventArgs e)
        {
            axWindowsMediaPlayer1.Ctlcontrols.pause();
        }

        private void button5_Click(object sender, EventArgs e)
        {
            axWindowsMediaPlayer1.Ctlcontrols.stop();
        }

        private void button1_Click(object sender, EventArgs e)
        {
            progressBar1.Minimum = 0;
            progressBar1.Maximum = 100;
            //MessageBox.Show("videofile = " + videofile + "\nintersection =" + intersection + "\nheading = "+heading);
            Process process = new Process();
            process.StartInfo.FileName = "Vehicle Tracking Intersection.exe";
            process.StartInfo.Arguments = "\""+videofile+"\" "+intersection +" "+heading;
            process.StartInfo.UseShellExecute = false;
            process.StartInfo.RedirectStandardOutput = true;
            process.StartInfo.CreateNoWindow = true;
            process.Start();
            while (!process.StandardOutput.EndOfStream)
            {
                string line = process.StandardOutput.ReadLine();
                int.TryParse(line,out pvalue);
                progressBar1.Value = pvalue;
            }
            //axWindowsMediaPlayer1.URL = videofile;
            axWindowsMediaPlayer1.URL = videofile + "_RESULT.avi";
            //axWindowsMediaPlayer1.URL = "E:/Movies/The Great Wall 2016 1080p.mp4";

            //string output = process.StandardOutput.ReadToEnd();
            //int.TryParse(output, progressBar1.Value);

            //process.WaitForExit();
            //StreamReader reader = process.StandardOutput;
            //string output = reader.ReadToEnd();
            //Console.WriteLine(output);

            //process.WaitForExit();
            //process.Close();
        }

        private void textBox1_TextChanged(object sender, EventArgs e)
        {
         
        }

        private void Form1_Load(object sender, EventArgs e)
        {

        }

        private void radioButton2_CheckedChanged(object sender, EventArgs e)
        {
            intersection = 1;
        }

        private void textBox2_TextChanged(object sender, EventArgs e)
        {
            int.TryParse(textBox2.Text, out heading);
        }

        private void progressBar1_Click(object sender, EventArgs e)
        {

        }

        private void axWindowsMediaPlayer1_Enter(object sender, EventArgs e)
        {

        }
    }
}
