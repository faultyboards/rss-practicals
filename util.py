import numpy as np


def np2tpl(narray):
	'''
	Utility function converting rank-1 numpy arrays into tuples
	'''
	return tuple([x for x in narray])

def tpl2np(intuple):
	'''
	Utility function converting tuples into rank-1 numpy arrays
	'''
	return np.array(intuple)