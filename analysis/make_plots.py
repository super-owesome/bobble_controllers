import os, matplotlib, argparse
matplotlib.use('agg')
import matplotlib.pyplot as plt
import seaborn as sns
plt.rcParams['axes.grid'] = True
sns.set_context("poster", font_scale=1.1)
sns.set_palette("Dark2")

def load_data(sim_data_bag_file, run_name):
    # Parse sim data
    from analysis_tools.parsing import parse_single_run
    print "Loading sim data from run : " + str(run_name) + " ... "
    df = {}
    df[run_name] = parse_single_run(sim_data_bag_file)
    print "Success."
    # Parse plotting config
    from analysis_tools.parsing import parse_config_file
    pc = parse_config_file("plots.yaml")
    print "Success."
    return df, pc

def make_plots(df, pc, run_name, out_dir):
    from analysis_tools.plots import desired_vs_actual_for_runs
    # Make the Tilt plot
    fig = plt.figure(figsize=(20, 10), dpi=40)
    ax1 = fig.add_subplot(111)
    pc['tilt_control']['runs'] = [run_name]
    desired_vs_actual_for_runs(ax1, df, pc['tilt_control'])
    fig.tight_layout()
    fig_file_name = os.path.join(out_dir, 'TiltControl.png')
    plt.savefig(fig_file_name, bbox_inches='tight')
    # Make the Velocity plot
    fig = plt.figure(figsize=(20, 10), dpi=40)
    ax1 = fig.add_subplot(111)
    pc['velocity_control']['runs'] = [run_name]
    desired_vs_actual_for_runs(ax1, df, pc['velocity_control'])
    fig.tight_layout()
    fig_file_name = os.path.join(out_dir, 'VelocityControl.png')
    plt.savefig(fig_file_name, bbox_inches='tight')

def main():
    argParser = argparse.ArgumentParser()
    default_run = "impulse_force.bag"
    argParser.add_argument("-r",  "--run", default=default_run, help="Bag file for simulation run of interest")
    argParser.add_argument("-o",  "--out", default=".", help="Output directory.")
    args = argParser.parse_args()
    run_name = os.path.splitext(args.run)[0]
    df, pc = load_data(args.run, run_name)
    # Pass the data and plotting configuration into the make_plots function
    make_plots(df, pc, run_name, args.out)

if __name__ == "__main__":
    main()



