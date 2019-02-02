import matplotlib.lines as mlines
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.signal import freqz
from filters import *


def apply_common_plot_styles(ax, pc):
    if 'legend' in pc.keys():
        legend_list = []
        for key in pc['legend']:
            data_key = mlines.Line2D([], [], color=pc['legend'][key]['color'], linestyle=pc['legend'][key]['linestyle'], label=key)
            legend_list.append(data_key)
        if 'legend_loc' in pc.keys():
            ax.legend(handles=legend_list, loc=pc['legend_loc'])
        else:
            ax.legend(handles=legend_list, loc='lower right')
    ax.set_title(pc['title'])
    ax.set_xlabel(pc['x_label'])
    ax.set_ylabel(pc['y_label'])
    if 'xlim' in pc.keys():
        ax.set_xlim(pc['xlim'])
    if 'ylim' in pc.keys():
        ax.set_ylim(pc['ylim'])
    ax.grid(True)
    plt.tick_params(axis='both', which='both', top=False, right=False)


def coplot_var_for_runs(axis, data, pc):
    if 'x_var' in pc.keys():
        for index, run in enumerate(pc['runs']):
            if 'linestyles' in pc.keys():
                data[run].plot(x=pc['x_var'], y=pc['y_var'], ax=axis, color=pc['colors'][index],
                               linestyle=pc['linestyles'][index], label='_nolegend_')
            else:
                data[run].plot(x=pc['x_var'], y=pc['y_var'], ax=axis, color=pc['colors'][index], label='_nolegend_')
    else:
        for index, run in enumerate(pc['runs']):
            if 'linestyles' in pc.keys():
                data[run][pc['y_var']].plot(ax=axis, color=pc['colors'][index],
                                            linestyle=pc['linestyles'][index], label='_nolegend_')
            else:
                data[run][pc['y_var']].plot(ax=axis, color=pc['colors'][index], label='_nolegend_')
    apply_common_plot_styles(axis, pc)


def coplot_vars_for_run(axis, data, pc):
    for index, var in enumerate(pc['vars']):
        if 'linestyles' in pc.keys():
            data[pc['run']][var].plot(ax=axis, color=pc['colors'][index],
                                      linestyle=pc['linestyles'][index], label='_nolegend_')
        else:
            data[pc['run']][var].plot(ax=axis, color=pc['colors'][index], label='_nolegend_')
    apply_common_plot_styles(axis, pc)


def desired_vs_actual_for_runs(axis, data, pc):
    if 'desired_x_var' in pc.keys():
        for index, run in enumerate(pc['runs']):
            data[run].plot(x=pc['actual_x_var'], y=pc['actual_y_var'], ax=axis, color=pc['colors'][index+1],
                           label='_nolegend_')
            data[run].plot(x=pc['desired_x_var'], y=pc['desired_y_var'], ax=axis, color=pc['colors'][index],
                           label='_nolegend_')
    else:
        for index, run in enumerate(pc['runs']):
            data[run][pc['actual_y_var']].plot(ax=axis, color=pc['colors'][index+1],
                                                label='_nolegend_')
            data[run][pc['desired_y_var']].plot(ax=axis, color=pc['colors'][index],
                                                label='_nolegend_')
    apply_common_plot_styles(axis, pc)


def make_static_plot(df, pc, plot_name, plot_func=coplot_var_for_runs, savefig=False, dpi=40, height=20, width=10):
    fig = plt.figure(figsize=(height, width), dpi=dpi)
    ax1 = fig.add_subplot(111)
    plot_func(ax1, df, pc)
    fig.tight_layout()
    sns.despine()
    if savefig:
        plt.savefig(plot_name+'.png', bbox_inches='tight')


def lowpass_design_for_run(df, pc, plot_name, savefig=False, dpi=40, height=20, width=10):
    fs = pc['sample_rate']
    cutoff_freq = pc['cutoff_freq']
    order = pc['filter_order']
    raw_signal = df[pc['run']][pc['raw_signal']]
    period = 1.0/fs

    fig1 = plt.figure(figsize=(width, height), dpi=dpi)
    ax1 = fig1.add_subplot(111)
    # The FFT of the signal
    sig_fft = scipy.fftpack.fft(raw_signal)
    # And the power (sig_fft is of complex dtype)
    power = np.abs(sig_fft)
    # The corresponding frequencies
    sample_freq = scipy.fftpack.fftfreq(raw_signal.size, d=period)
    # Find the peak frequency: we can focus on only the positive frequencies
    pos_mask = np.where(sample_freq > 0)
    freqs = sample_freq[pos_mask]
    power = power[pos_mask]
    # Plot the FFT power
    ax1.plot(freqs, power)
    ax1.grid(True)
    ax1.set_xlabel('Frequency [Hz]')
    ax1.set_ylabel('Power')
    ax1.set_title('Frequency Spectrum')

    fig1 = plt.figure(figsize=(width, height), dpi=dpi)
    ax1 = fig1.add_subplot(111)
    filtered_sig = butter_lowpass_filter(raw_signal, fs, cutoff_freq, order=order)
    ax1.plot(raw_signal.index, raw_signal, label='Raw signal')
    ax1.plot(raw_signal.index, filtered_sig, label='Filtered signal')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel(pc['y_label'])
    ax1.grid(True)
    ax1.legend(loc='best')
    ax1.set_title('Raw vs Filtered')

    fig2 = plt.figure(figsize=(width, height), dpi=dpi)
    ax1 = fig2.add_subplot(211)
    ax2 = fig2.add_subplot(212)
    b, a = butter_lowpass(cutoff_freq, fs, order=order)
    w, h = freqz(b, a, worN=2000)
    ax1.plot((fs * 0.5 / np.pi) * w, abs(h))
    ax1.plot(cutoff_freq, 0.5*np.sqrt(2), 'ko')
    ax1.axvline(cutoff_freq, color='k')
    ax1.set_title('Magnitude Response')
    ax1.set_xlim(0, 0.5*fs)
    ax1.set_xlabel('Frequency (Hz)')
    ax1.set_ylabel('Magnitude')
    ax1.grid(True)
    ax1.legend(loc='best')

    angles = np.angle(h, deg=True)
    ax2.plot((fs * 0.5 / np.pi) * w, angles)
    ax2.plot(cutoff_freq, 0.5*np.sqrt(2), 'ko')
    ax2.axvline(cutoff_freq, color='k')
    ax2.set_title('Phase Response')
    ax2.set_xlim(0, 0.5*fs)
    ax2.set_ylim(-200.0, 200.0)
    ax2.set_xlabel('Frequency (Hz)')
    ax2.set_ylabel('Phase (deg)')
    ax2.grid(True)
    ax2.legend(loc='best')

    plt.tight_layout()

    if savefig:
        plt.savefig(plot_name+'.png', bbox_inches='tight')

