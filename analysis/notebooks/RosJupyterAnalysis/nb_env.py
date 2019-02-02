# Establish the analysis environment used with Jupyter Notebook
import os, sys
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings("ignore")
import seaborn as sns
import pandas as pd
import numpy as np
import scipy
import scipy.fftpack
import math
try:
    from analysis_tools.parsing import *
    from analysis_tools.plots import *
    from analysis_tools.filters import *
except ImportError:
    sys.path.append(os.path.join('..', '..', '..', 'src'))
    from analysis_tools.parsing import *
    from analysis_tools.plots import *
    from analysis_tools.filters import *

# Set some global plotting configs. We want these for all plots
sns.set_context("poster", font_scale=1.1)

# Set paths to relevant data directories and config files
data_home = os.path.join(os.getcwd(), "data")
print("Reading all bag files in directory : ")
print(data_home)

# Load configs and data
df = parse_all_runs_in_dir(data_home)
print("Successfully loaded runs : ")

