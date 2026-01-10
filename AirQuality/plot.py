#!/usr/bin/env python3
"""
Plot air quality data from Teensy box and EPAM sensor.
"""


import itertools
import os
import warnings
import matplotlib as mpl
import matplotlib.pyplot as plt
import pandas as pd


def to_datetime_ignore_warnings(data):
    """
    Suppress warnings when converting datetime data.
    """
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', category=UserWarning)
        return pd.to_datetime(data)


def read_box_file(file):
    """
    Read file from Teensy box.
    """
    df = pd.read_csv(file)
    df['Timestamp'] = to_datetime_ignore_warnings(df['Timestamp'])
    return df


def read_epam_file(file, tz_offset=0):
    """
    Read file from EPAM sensor. Optionally adjust the timezone.
    """
    df1 = pd.read_csv(file, nrows=8, names=['Field', 'Value'])
    assert df1['Field'].iloc[2] == 'Date:', 'Date field not found'
    date = df1['Value'].iloc[2]

    df2 = pd.read_csv(file, skiprows=9, names=[
                      'Unused1', 'Timestamp', 'PM2.5_epam', 'Unused2'])
    df2 = df2.drop(columns=['Unused1', 'Unused2'])
    df2['Timestamp'] = to_datetime_ignore_warnings(
        date + ' ' + df2['Timestamp'])
    df2['Timestamp'] += pd.Timedelta(hours=tz_offset)  # adjust timezone
    df2['PM2.5_epam'] *= 1e3  # convert milligrams to micrograms
    return df2


def read_files(directory, box_files, epam_files, resample_interval):
    """
    Read and join files from Teensy box and EPAM sensor.
    Resample and average data using the specified time interval.
    """
    df1 = pd.concat([read_box_file(directory + file) for file in box_files])
    df1 = df1.resample(resample_interval, on='Timestamp').mean()
    if len(epam_files) > 0:
        df2 = pd.concat([read_epam_file(directory + file)
                        for file in epam_files])
        df2 = df2.resample(resample_interval, on='Timestamp').mean()
        df1 = df1.join(df2, how='outer', sort=True, validate='one_to_one')
    return df1.reset_index()


if __name__ == '__main__':

    # Directory containing data files.
    DIRECTORY = os.path.expanduser('~/Downloads/')

    # Name(s) of Teensy box file(s). Must not be empty.
    BOX_FILES = [
        'Box1pm25_2025-10-11.csv']

    # Name(s) of EPAM sensor file(s). May be empty.
    EPAM_FILES = [
        'Epam_2025-10-11.csv']

    # Names of data columns to plot.
    COLUMNS = [
        'PM2.5_1MinAvg',
        'PM2.5_env',
        'PM2.5_epam']

    # X-axis label.
    XLABEL = 'Month-Day Hour'

    # Y-axis label.
    YLABEL = 'PM Concentration (µg/m³)'

    df = read_files(DIRECTORY, BOX_FILES, EPAM_FILES, '5min')

    MARKER_SIZE = 4.0
    color = itertools.cycle(mpl.color_sequences['Set1'])
    ax = None
    for y in COLUMNS:
        if y in df:
            ax = df.plot.scatter(
                x='Timestamp', y=y, color=next(color), s=MARKER_SIZE, ax=ax)
        else:
            print(f'Warning: Column "{y}" not found in data.')
    ax.grid(True)
    ax.legend(COLUMNS)
    ax.set_xlabel(XLABEL)
    ax.set_ylabel(YLABEL)
    plt.show()
