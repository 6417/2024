import csv
import json

from wpimath import geometry, units
import numpy as np


filename = "tag_pos.csv"

json_data = {"fiducials" : []}

csv_file = open(filename, "r")
reader = csv.reader(csv_file, delimiter=",")

#field_bright = 8.21 in meters
#field_with = 16.54

field_bright = 323.25 
field_with = 651.25

for row in reader:
	id = int(row[0])
	x = float(row[1]) - field_with/2
	y = float(row[2]) - field_bright/2
	z = float(row[3])
	rotation = float(row[4])
	
	q = geometry.Rotation3d(0,0,units.degreesToRadians(rotation)).getQuaternion()
	
	json_data["fiducials"].append(
				{
		"family": "apriltag3_36h11_classic",
		"id": id,
		"size": 160.0,
		"transform" : [
			round(q.W(),5),
			0,
			0,
			round(units.inchesToMeters(x),5),
			0,
			round(q.X(),5),
			0,
			round(units.inchesToMeters(y),5),
			0,
			0,
			round(q.Y(),5),
			round(units.inchesToMeters(z),5),
			0,
			0,
			0,
			round(q.Z(),5)
		],
		"unique" : 1,
		},
	)

f = open("result_json.fmap","w")

json.dump(json_data, f, indent=2)