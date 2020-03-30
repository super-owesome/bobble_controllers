import unittest
import os

import rospkg
PKG="bobble_controllers"

class ParsingTest(unittest.TestCase):

    def setUp(self):
        rospack = rospkg.RosPack()
        self.sample_data_path = os.path.join(rospack.get_path("bobble_controllers"), "test", "data")
        self.test_dir = os.path.join(rospack.get_path("bobble_controllers"), "test")
        self.run_name = 'no_balance'
        return

    def test_ParseConfigFile(self):
        from analysis_tools.parsing import parse_config_file
        plot_config = os.path.join(self.test_dir, 'plots.yaml')
        pc = parse_config_file(plot_config)
        self.assertEquals(pc['measured_tilt']['title'], 'Tilt Angle')
        return

    def test_ParseSingleBagFile(self):
        from analysis_tools.parsing import parse_single_run
        sim_run = os.path.join(self.sample_data_path, self.run_name+'.bag')
        df = parse_single_run(sim_run)
        final_tilt = df['Tilt'].iloc[-1]
        print final_tilt
        #self.assertTrue(abs(final_tilt) > 45.0)
        return

    def test_ParseManyBagFiles(self):
        from analysis_tools.parsing import parse_all_runs_in_dir
        sim_data_dir = self.sample_data_path
        df = parse_all_runs_in_dir(sim_data_dir, csv_convert=True)
        final_tilt = df[self.run_name]['Tilt'].iloc[-1]
        print final_tilt
        #self.assertTrue(abs(final_tilt) > 45.0)
        return

    def test_ParseManyCsvFiles(self):
        from analysis_tools.parsing import parse_all_csv_runs_in_dir
        sim_data_dir = self.sample_data_path
        df = parse_all_csv_runs_in_dir(sim_data_dir)
        final_tilt = df[self.run_name]['Tilt'].iloc[-1]
        print final_tilt
        #self.assertTrue(abs(final_tilt) > 45.0)
        return

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'parsings_test', ParsingTest)

