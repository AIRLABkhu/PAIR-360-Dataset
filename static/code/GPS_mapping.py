import pandas as pd
import folium
import os
import argparse

parser = argparse.ArgumentParser(description='Create map by using GPS.')
parser.add_argument('--path',  metavar='folder', default='/media/air/My Passport1/AIR360/Nov_dataset_1109', nargs='?', help='Data folder')
parser.add_argument('--sat', type=bool, default=True, help='configureation: satelite background or not')
parsed = parser.parse_args()


"""_summary_
Structure:
    /path/{Area1}/{Seq1}/GPS.csv
    /path/{Area1}/{Seq2}/GPS.csv
    ...
    /path/{Area2}/{Seq1}/GPS.csv
     ...

Returns:
    png : fisheye image projected by paired pcd.
"""

#To paint the same color for each area
area_color_list = [('College_of_Life_Scince', '#000000', "black"),
	            ('College_of_Physical_Education', '#DB8291', 'pink'), 
	            ('2nd_Dormitory','#FF0000', 'red'),
	            ('1st_Dormitory', '#FFA500', 'orange'), 
	            ('College_of_Engineering','#00FF00','lime'), 
	            ('Parking_Peace_Amphitheater','#0000FF', 'blue'), 
	            ('Central_Library', '#800080', 'purple')]

#file_color_list = ['#000000', '#DB8291', '#FF0000', '#FFA500', '#00FF00', '#0000FF', '#0000FF', '#800080']

folder = parsed.path
center = [37.2425, 127.083484] #South Korea Base 
file_name = 'GPS.csv'

if sat:
    # 구글 지도 타일 설정
    tiles = "http://mt0.google.com/vt/lyrs=s&hl=ko&x={x}&y={y}&z={z}"
    # 속성 설정
    attr = "Google"
    # 지도 객체 생성
    m = folium.Map(location = center,
                zoom_start = 10,
                tiles = tiles,
                attr = attr)
else:
    m = folium.Map(location=center, tiles='OpenStreetMap', zoom_start=17)

for area_path in area_color_list:
    for folder_path, _, files in os.walk(os.path.join(folder, area_path[0])):

        # Current folder path
        print("current folder :", folder_path)

        if file_name in files:
            df = pd.read_csv(os.path.join(folder_path, file_name))  # read GPS.csv
            df['longitude'] = pd.to_numeric(df['longitude(deg)'])
            df['latitude'] = pd.to_numeric(df['latitude(deg)'])
            lines = df[['latitude','longitude']]
            pos_list = lines[['latitude','longitude']].values[:].tolist()
            folium.PolyLine(
                locations = pos_list,
                tooltip = 'PolyLine',
                color = area_path[1]
            ).add_to(m)
        else:
            continue  # move next folder
m.save(os.path.join(folder,'sat_gps_map.html'))

