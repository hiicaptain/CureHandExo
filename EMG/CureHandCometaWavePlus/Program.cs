using System;
using System.Windows.Forms;

namespace Waveplus.CureHand
{
    static class Program
    {
        private static WaveplusDaqForm _mainForm;

        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
                
        static void Main()
        {            
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            try
            {
                _mainForm = new WaveplusDaqForm();
                Application.Run(_mainForm);
            }
            catch(Exception e)
            {
                MessageBox.Show(e.Message);
            }
            finally
            {
                if (_mainForm._daqSystem != null) _mainForm._daqSystem.Dispose();
                Application.Exit();
                Environment.Exit(0);
            }
        }
     }
}