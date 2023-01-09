# -----------------------------------------------------------------------------
# Project "5.1: Control and Trajectory Tracking for Autonomous Vehicles"
# Authors     : Munir Jojo-Verge, Aaron Brown, Mathilda Badoual.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file: Plots the throttle and steering error values from the
#                       PID controller response.
# -----------------------------------------------------------------------------

import matplotlib as mpl
mpl.rc('font', family='Times New Roman')
import matplotlib.pyplot as plt
import pandas as pd


def read_steer_data():
    """Reads in the text file of steering commands into a Pandas DataFrame."""
    steer_file = 'steer_pid_data.txt'
    steer_df = pd.read_csv(
        steer_file, 
        delim_whitespace=True, 
        header=None, 
        usecols=[0, 1, 2]
    )
    steer_df.columns = [
        'Time-step', 'Error Steering', 'Steering Output'
    ]
    print(f'Steer data:\n{steer_df.head()}\n')
    return steer_df


def read_throttle_data():
    """Reads in the text file of throttle commands into a Pandas DataFrame."""
    throttle_file = 'throttle_pid_data.txt'
    throttle_df = pd.read_csv(
        throttle_file, 
        delim_whitespace=True, 
        header=None,
        usecols=[0, 1, 2, 3]
    )
    throttle_df.columns = [
        'Time-step', 'Error Throttle', 'Brake Output', 'Throttle Output'
    ]
    print(f'Throttle data:\n{throttle_df.head()}\n')
    return throttle_df


def plot_steer_data(
        steer_df: pd.DataFrame, 
        n_rows: int
):
    """Plots the steering command data as a Matplotlib figure instance.
    
    :param steer_df: Pandas DataFrame containing the steering command data.
    :param n_rows: Number of rows in the DataFrame to plot,
        corresponds to the number of time-steps of the experiment run.
    """

    steer_df2 = steer_df[:n_rows]
    steer_df2.plot(
        x=steer_df.columns[0], 
        y=[steer_df.columns[1], 
           steer_df.columns[2]
        ], 
        kind='line', 
        figsize=(24, 20))
    plt.legend(loc='lower right', fontsize='xx-large')
    plt.suptitle('PID Controller for Trajectory Tracking — Steering', fontsize=24)
    txt_s = r'$K_{p} = 0.3$,  $K_{i} = 0.0025$, $K_{d}=0.17$,  $\alpha_{max} = 0.6$,  $\alpha_{min} = -0.6$'
    plt.title(txt_s, fontsize=16)
    plt.tight_layout(pad=3.5)
    plt.xlabel(steer_df.columns[0], fontsize=20)
    plt.show()
 
    
def plot_throttle_data(
        throttle_df: pd.DataFrame, 
        n_rows: int
):
    """Plots the throttle command data as a Matplotlib figure instance.
    
    :param throttle_df: Pandas DataFrame containing the throttle command data.
    :param n_rows: Number of rows in the DataFrame to plot,
        corresponds to number of time-steps of the experiment run.
    """

    throttle_df2 = throttle_df[:n_rows]
    throttle_df2.plot(
        x=throttle_df.columns[0], 
        y=[throttle_df.columns[1], 
            throttle_df.columns[2], 
            throttle_df.columns[3]
        ],
        kind='line', 
        figsize=(24, 20)
    )
    plt.legend(
        loc='upper right', 
        fontsize='xx-large'
    )
    plt.suptitle(
        'PID Controller For Trajectory Tracking — Throttle',
        fontsize=24
    )
    txt_t = r'$K_{p} = 0.21$,  $K_{i} = 0.0006$,  $K_{d} = 0.080$,  $\alpha_{max} = 1.0$,  $\alpha_{min} = -1.0$'
    plt.title(txt_t, fontsize=16)
    plt.tight_layout(pad=3.5)
    plt.xlabel(throttle_df.columns[0], fontsize=20)
    plt.show()


if __name__ == '__main__':
    steer_df = read_steer_data()
    throttle_df = read_throttle_data()
    # Use all rows in text file for Matplotlib figure 
    n_rows = -1 #2000
    plot_steer_data(steer_df, n_rows)
    plot_throttle_data(throttle_df, n_rows)