import os
import cv2, argparse
import numpy as np

parser = argparse.ArgumentParser(description='Camera Calibration using OpenCV. Supported config files: LSD, DSO, ORB')

parser.add_argument('-i', action='store', default='./', dest='input', help='Image Directory (Default: current directory)')
parser.add_argument('-o', action='store', default='./calib', dest='output', help='Output file (Default: Determined by type)')
parser.add_argument('-t', action='store', default='lsd', dest='type', help='Output file format (Ex. LSD formats calib.cfg, etc.)')
parser.add_argument('-cw', action='store', default=6, dest='width', help='Checkerboard Width (Default: 6)')
parser.add_argument('-ch', action='store', default=9, dest='height', help='Checkerboard Height (Default: 9)')
parser.add_argument('-iw', action='store', default=640, dest='img_width', help='Width written to config (Default: 640)')
parser.add_argument('-ih', action='store', default=480, dest='img_height', help='Height written to config (Default: 480)')
parser.add_argument('-sq_size', action='store', default=25, dest='square_size', help='Checkerboard square width in millimeters (Default: 25)')
parser.add_argument('--debug', action='store_true', help='Enable debug output')

def main():
	try:
		args = parser.parse_args()
		
		# Define critera for corner drawing
		criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, args.square_size, 0.001)
		
		if type(args.input) is not str:
			raise Exception("Error: Input Directory argument (-i) should be a string.")
		if type(args.output) is not str:
			raise Exception("Error: Output Directory argument (-o) should be a string.")

		if type(args.type) is not str:
			raise Exception("Error: Type argument (-t) should be a string.")
			
		# Set of images to perform calibration on
		img_calib_set = []
		# Initialize 3D points
		points_3d_sample = np.zeros((args.width*args.height,3), np.float32)
		points_3d_sample[:,:2] = np.mgrid[0:args.height, 0:args.width].T.reshape(-1,2)
	
		# Initialize containers
		points_2d = []
		points_3d = []
		
		# Collect files from input directory
		for img_calib in os.listdir(args.input):
			if args.debug:
				print "Loaded file: %s%s" % (args.input, img_calib)
			img_calib = cv2.imread(args.input + '/' + img_calib, 0)
			img_calib_set.append(img_calib)
		
		if args.debug:
			cv2.namedWindow('Checkerboard Corners', cv2.WINDOW_NORMAL)
			cv2.resizeWindow('Checkerboard Corners',1280,720)
			cv2.startWindowThread()
		i = 1
		
		# Iterate through the images and attempt to find chessboard corners
		for img in img_calib_set:
			
			# Find the chessboard corners
			ret_val, corner = cv2.findChessboardCorners(img, (args.height, args.width), None)
		
			if ret_val == True:
				points_3d.append(points_3d_sample)
				
				# Increase accuracy of corners with cornerSubPix()
				cv2.cornerSubPix(img, corner, (11,11), (-1,-1), criteria)
				points_2d.append(corner)
				
				if args.debug:
					img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
					cv2.drawChessboardCorners(img, (9,6), corner, ret_val)
					cv2.imshow('Checkerboard Corners',img)
					cv2.waitKey()
					print "Done processing image %d" % i
			i += 1
		if args.debug:
			cv2.waitKey(1)
			cv2.destroyAllWindows()
			cv2.waitKey(1)
		print "Image processing complete. Calibrating, this may take some time..."
		# Calculate camera properties
		# ret_val = Bool
		# int_mat = Matrix (intrinsic)
		# dist_coef = Matrix (distortion)
		# rot_vector = Matrix (rotation)
		# tran_vector = Matrix (translation)
		ret_val, int_mat, dist_coef, rot_vector, tran_vector = cv2.calibrateCamera(points_3d, points_2d, img_calib_set[0].shape[::-1], None, None)
		
		# Make sure file is correct type for chosen SLAM type
		args.type = args.type.lower()
		if args.type == 'lsd' and args.output[-4:len(args.output)] != '.cfg':
			args.output += '.cfg'
		elif args.type == 'dso' and args.output[-4:len(args.output)] != '.txt':
			args.output += '.txt'
		elif args.output[-5:len(args.output)] != '.yaml':
			args.output += '.yaml'
		
		########################################################################
		############### Define higher-order function for easy exit #############
		########################################################################
		def exit():
			print '-'*40
			print("Calibration file written to %s\n" % (args.output))
			print "Calibration values"
			print "fx : %0.6f" % int_mat[0][0]
			print "fy : %0.6f" % int_mat[1][1]
			print "cx : %0.6f" % int_mat[0][2]
			print "cy : %0.6f" % int_mat[1][2]
			print "k1 : %0.6f" % dist_coef[0][0]
			print "k2 : %0.6f" % dist_coef[0][1]
			print "p1 : %0.6f" % dist_coef[0][2]
			print "p2 : %0.6f" % dist_coef[0][3]
			if len(dist_coef) > 4:
				print "k3 : %0.6f" % dist_coef[0][4]
		
				# Calculate reprojection error (should be less than 0.5)
			mean_err = 0
			for i in xrange(len(points_3d)):
				points_2d2, _ = cv2.projectPoints(points_3d[i], rot_vector[i], tran_vector[i], int_mat, dist_coef)
				error = cv2.norm(points_2d[i], points_2d2, cv2.NORM_L2)/len(points_2d2)
				mean_err += error
			print "Total reprojection error: ", mean_err/len(points_3d)
		
		##########################################################################
		##########################################################################
		
		
		
		with open(args.output, 'w') as f:
			fline = ""
			# Change file based on target calibration configuration type
			if args.type == 'lsd':
				fline = "%0.10f %0.10f %0.10f %0.10f %0.10f %0.10f %0.10f %0.10f\n" % (int_mat[0][0], int_mat[1][1], int_mat[0][2], int_mat[1][2], dist_coef[0][0], dist_coef[0][1], dist_coef[0][2], dist_coef[0][3])
			elif args.type == 'dso':
				fline = "Pinhole %0.10f %0.10f %0.10f %0.10f 0\n" % (int_mat[0][0], int_mat[1][1], int_mat[0][2], int_mat[1][2])
			else:
				f.write(ORB_YAML % (int_mat[0][0], int_mat[1][1], int_mat[0][2], int_mat[1][2], dist_coef[0][0], dist_coef[0][1], dist_coef[0][2], dist_coef[0][3]))
				return exit()
				
			f.write(fline)
			w_h = "%d %d\n" % (args.img_width, args.img_height)
			f.write(w_h)
			f.write("crop\n")
			f.write(w_h)
			
			return exit()

	except Exception as e:
		print e

ORB_YAML = "\
%%YAML:1.0\n \
\n\
#-------------------------------------------------------------------------------------------- \n\
# Camera Parameters. Adjust them! \n\
#-------------------------------------------------------------------------------------------- \n\
\n\
# Camera calibration and distortion parameters (OpenCV) \n\
Camera.fx: %0.10f \n\
Camera.fy: %0.10f \n\
Camera.cx: %0.10f \n\
Camera.cy: %0.10f \n\
\n\
Camera.k1: %0.10f \n\
Camera.k2: %0.10f \n\
Camera.p1: %0.10f \n\
Camera.p2: %0.10f \n\
\n\
# Camera frames per second \n\
Camera.fps: 60.0 \n\
\n\
# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale) \n\
Camera.RGB: 1 \n\
\n\
#-------------------------------------------------------------------------------------------- \n\
# ORB Parameters \n\
#-------------------------------------------------------------------------------------------- \n\
\n\
# ORB Extractor: Number of features per image \n\
ORBextractor.nFeatures: 2000 \n\
\n\
# ORB Extractor: Scale factor between levels in the scale pyramid \n\
ORBextractor.scaleFactor: 1.2 \n\
\n\
# ORB Extractor: Number of levels in the scale pyramid \n\
ORBextractor.nLevels: 8 \n\
\n\
# ORB Extractor: Fast threshold \n\
# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response. \n\
# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST \n\
# You can lower these values if your images have low contrast \n\
ORBextractor.iniThFAST: 20 \n\
ORBextractor.minThFAST: 7 \n\
\n\
#-------------------------------------------------------------------------------------------- \n\
# Viewer Parameters \n\
#-------------------------------------------------------------------------------------------- \n\
Viewer.KeyFrameSize: 0.1 \n\
Viewer.KeyFrameLineWidth: 1 \n\
Viewer.GraphLineWidth: 1 \n\
Viewer.PointSize:2 \n\
Viewer.CameraSize: 0.15 \n\
Viewer.CameraLineWidth: 2 \n\
Viewer.ViewpointX: 0 \n\
Viewer.ViewpointY: -10 \n\
Viewer.ViewpointZ: -0.1 \n\
Viewer.ViewpointF: 2000 \n\
"

if __name__ == "__main__":
	main()
	




