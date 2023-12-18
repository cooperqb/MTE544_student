import matplotlib
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from utilities import FileReader

def plot_errors(filename, title):
    # imu_ax, imu_ay, odom_v, odom_w, kf_ax, kf_ay, kf_vx, kf_w, x, y, stamp, 
    headers, values=FileReader(filename).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    matplotlib.rcParams.update({'font.size': 22})
    fig = plt.figure(layout="constrained", figsize=(30, 5))

    gs = GridSpec(1, 5, figure=fig)
    
    if title:
        fig.suptitle(title)

    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot([lin[6] for lin in values], [lin[7] for lin in values], label="KF Path")
    ax1.plot([lin[8] for lin in values], [lin[9] for lin in values], label="Odom Path")
    ax1.set_title("Sensor vs Filtered State Space")
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("y [m]")
    ax1.grid()
    ax1.set_aspect("equal")

    ax2 = fig.add_subplot(gs[0, 1])
    ax2.set_title("Sensor vs Filtered (ax)")
    ax2.plot(time_list, [lin[0] for lin in values], label=headers[0])
    ax2.plot(time_list, [lin[2] for lin in values], label=headers[2])
    ax2.set_ylabel("Value [$m/s^2$]")
    ax2.set_xlabel("Time [ns]")
    ax2.legend()
    ax2.grid()

    ax3 = fig.add_subplot(gs[0, 2])
    ax3.set_title("Sensor vs Filtered (ay)")
    ax3.plot(time_list, [lin[1] for lin in values], label=headers[1])
    ax3.plot(time_list, [lin[3] for lin in values], label=headers[3])
    ax3.set_ylabel("Value [$m/s^2$]")
    ax3.set_xlabel("Time [ns]")
    ax3.legend()
    ax3.grid()

    ax4 = fig.add_subplot(gs[0, 3])
    ax4.set_title("Sensor vs Filtered (x)")
    ax4.plot(time_list, [lin[6] for lin in values], label=headers[6])
    ax4.plot(time_list, [lin[8] for lin in values], label=headers[8])
    ax4.set_ylabel("Value [m]")
    ax4.set_xlabel("Time [ns]")
    ax4.legend()
    ax4.grid()

    ax5 = fig.add_subplot(gs[0, 4])
    ax5.set_title("Sensor vs Filtered (y)")
    ax5.plot(time_list, [lin[7] for lin in values], label=headers[6])
    ax5.plot(time_list, [lin[9] for lin in values], label=headers[8])
    ax5.set_ylabel("Value [m]")
    ax5.set_xlabel("Time [ns]")
    ax5.legend()
    ax5.grid()

    # saving
    f_save = filename.replace(".csv", ".png")
    plt.savefig(f_save)


import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    parser.add_argument('--titles', nargs='+', required=False, help='List of titles corresponding to files')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)
    print("with names", args.titles)

    filenames=args.files
    titles = args.titles
    if not titles:
        titles = [None] * len(filenames)
    assert len(filenames) == len(titles)
    for filename, title in zip(filenames, titles):
        plot_errors(filename, title)