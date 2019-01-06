import unittest
import os
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
import seaborn as sns
# Set some global plotting configs. We want these for all plots
plt.rcParams['axes.grid'] = True
sns.set_context("poster", font_scale=1.1)
sns.set_palette("Dark2")

import rospkg
PKG="bobble_controllers"

class PlotsTest(unittest.TestCase):

    def setUp(self):
        rospack = rospkg.RosPack()
        self.sample_data_path = os.path.join(rospack.get_path("bobble_controllers"), "test", "data")
        self.test_dir = os.path.join(rospack.get_path("bobble_controllers"), "test")
        return

    def helper_load_data(self):
        from analysis_tools.parsing import parse_config_file, parse_all_runs_in_dir
        # Parse configs
        plot_config = os.path.join(self.test_dir, 'plots.yaml')
        pc = parse_config_file(plot_config)
        # Parse data
        sim_runs_data_path = self.sample_data_path
        return parse_all_runs_in_dir(sim_runs_data_path), pc

    def test_PlotManyRunsTilt(self):
        from analysis_tools.plots import coplot_var_for_runs
        df, pc = self.helper_load_data()
        # Make the plot
        fig = plt.figure(figsize=(20, 10), dpi=40)
        ax1 = fig.add_subplot(111)
        coplot_var_for_runs(ax1, df, pc['measured_tilt'])
        fig.tight_layout()
        fig_file_name = os.path.join(self.test_dir, 'MeasuredTilt.png')
        plt.savefig(fig_file_name, bbox_inches='tight')

    def test_PlotManyRunsVelocity(self):
        from analysis_tools.plots import coplot_var_for_runs
        df, pc = self.helper_load_data()
        # Make the plot
        fig = plt.figure(figsize=(20, 10), dpi=40)
        ax1 = fig.add_subplot(111)
        coplot_var_for_runs(ax1, df, pc['velocity'])
        fig.tight_layout()
        fig_file_name = os.path.join(self.test_dir, 'MeasuredVelocity.png')
        plt.savefig(fig_file_name, bbox_inches='tight')

    def test_PlotDesiredVsActual(self):
        from analysis_tools.plots import desired_vs_actual_for_runs
        df, pc = self.helper_load_data()
        # Make the Tilt plot
        fig = plt.figure(figsize=(20, 10), dpi=40)
        ax1 = fig.add_subplot(111)
        desired_vs_actual_for_runs(ax1, df, pc['tilt_control'])
        fig.tight_layout()
        fig_file_name = os.path.join(self.test_dir, 'TiltControl.png')
        plt.savefig(fig_file_name, bbox_inches='tight')
        # Make the Velocity plot
        fig = plt.figure(figsize=(20, 10), dpi=40)
        ax1 = fig.add_subplot(111)
        desired_vs_actual_for_runs(ax1, df, pc['velocity_control'])
        fig.tight_layout()
        fig_file_name = os.path.join(self.test_dir, 'VelocityControl.png')
        plt.savefig(fig_file_name, bbox_inches='tight')


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'plots_test', PlotsTest)

