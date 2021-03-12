import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import ot.plot
from sklearn.preprocessing import StandardScaler
import os

def compare_features(feature_files):

  #hardcoded feature names should be loaded
  names = ["Average Vehicle Speed", "Average Vehicle Density", "Average Local Vehicle Density"]

  #load data
  # and create example density estimation of feature
  findex = 1
  fnames = []
  X = []

  sns.set_palette( sns.color_palette("tab10") )
  for f in feature_files:
    x = np.mean( np.load(f), axis=1)
    X.append( x )
    fname = os.path.basename(f)
    sns.kdeplot(x[:,findex], bw_method=0.1, label=fname)
    fnames.append(fname)

  # Add labels
  plt.xlabel( names[findex] )
  plt.ylabel("Probability Density")
  plt.legend(loc='upper center', fontsize='large')
  plt.show()

  n = X[0].shape[0]
  k = X[0].shape[1]
  a, b = np.ones((n,)) / n, np.ones((n,)) / n  # uniform distribution on samples

  D = np.zeros([len(X),len(X)])
  for i in range(2): #len(X)):
    for j in range(i+1,len(X)):
      scaler = StandardScaler()
      scaler.fit(X[i])
      Xi = scaler.transform( X[i] )
      scaler = StandardScaler()
      scaler.fit(X[j])
      Xj = scaler.transform( X[j] )
      M = ot.dist(Xi, Xj)
      A = ot.emd(a, b, M)
      D[i,j] = np.sqrt( np.sum( A * M ))
      D[j,i] = D[i,j]

  sns.heatmap(D, xticklabels=fnames, yticklabels=fnames)
  plt.show()


  #Example how to extract contribution from a optimal tansport map (Xi and A from last loop)
  Fd = np.zeros( Xi.shape[1] )
  for i in range(Xi.shape[1]):
    Fd[i] = np.sum( A * ot.dist( np.expand_dims(Xi[:,i],1),  np.expand_dims(Xj[:,i],1)) )

  plt.bar(x=range(len(Fd)), height=Fd/sum(Fd))
  plt.xticks(range(len(Fd)), names, rotation=15)
  plt.ylabel("Contribution To Total Distance")
  plt.show()

