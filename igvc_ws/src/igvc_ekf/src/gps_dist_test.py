import math

R = 6356    #6378.1 #Radius of the Earth
brng = math.radians(-1.62) #Bearing is converted to radians.
d = 12.37 / 1000 #Distance in km

#lat2  52.20444 - the lat result I'm hoping for
#lon2  0.36056 - the long result I'm hoping for.

lat1 = math.radians(35.210356)  #Current lat point converted to radians
lon1 = math.radians(-97.442451) #Current long point converted to radians

lat2 = math.asin( math.sin(lat1)*math.cos(d/R) +
     math.cos(lat1)*math.sin(d/R)*math.cos(brng))

lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(d/R)*math.cos(lat1),
             math.cos(d/R)-math.sin(lat1)*math.sin(lat2))

lat2 = math.degrees(lat2)
lon2 = math.degrees(lon2)

print(lat2)
print(lon2)

# import geopy
# from geopy.distance import VincentyDistance
# 
# # given: lat1, lon1, b = bearing in degrees, d = distance in kilometers
# d = 12.37 / 1000
# lat1 = 35.210356  #Current lat point converted to radians
# lon1 = -97.442451 #Current long point converted to radians
# b = 0
# 
# origin = geopy.Point(lat1, lon1)
# destination = VincentyDistance(kilometers=d).destination(origin, b)
# 
# lat2, lon2 = destination.latitude, destination.longitude