import math

# top-left reference point
p0 = {
    "scrX": 23.69,  # Minimum X position on screen
    "scrY": -0.5,  # Minimum Y position on screen
    "lat": 26.19111111,  # Latitude
    "lng": 91.69361111  # Longitude
}

# bottom-right reference point
p1 = {
    "scrX": 276,  # Maximum X position on screen
    "scrY": 178.9,  # Maximum Y position on screen
    "lat": 26.18916667,  # Latitude
    "lng": 91.69527778  # Longitude
}

radius = 6371  # Earth Radius in Km


# This function converts lat and lng coordinates to GLOBAL X and Y positions
def latlng_to_global_xy(lng, lat):
    # Calculates x based on cos of average of the latitudes
    x = radius * lng * math.cos((p0["lat"] + p1["lat"]) / 2)
    # Calculates y based on latitude
    y = radius * lat
    return {"x": x, "y": y}


# Calculate global X and Y for top-left reference point
p0["pos"] = latlng_to_global_xy(p0["lat"], p0["lng"])
# Calculate global X and Y for bottom-right reference point
p1["pos"] = latlng_to_global_xy(p1["lat"], p1["lng"])


# This function converts lat and lng coordinates to SCREEN X and Y positions
def latlng_to_screen_xy(lng, lat):
    # Calculate global X and Y for projection point
    pos = latlng_to_global_xy(lng, lat)
    # Calculate the percentage of Global X position in relation to total global width
    pos["perX"] = ((pos["x"] - p0["pos"]["x"]) / (p1["pos"]["x"] - p0["pos"]["x"]))
    # Calculate the percentage of Global Y position in relation to total global height
    pos["perY"] = ((pos["y"] - p0["pos"]["y"]) / (p1["pos"]["y"] - p0["pos"]["y"]))

    # Returns the screen position based on reference points
    return {
        "x": p0["scrX"] + (p1["scrX"] - p0["scrX"]) * pos["perX"],
        "y": p0["scrY"] + (p1["scrY"] - p0["scrY"]) * pos["perY"]
    }


# The usage is like this
import math

coordinates = [(91.69428490362668,26.18975333575588), (91.69547678726806,26.18933769709107), (91.69559333650771,26.19127552995775), (91.69438297328212,26.19132877077263), (91.69405718790988,26.19095153638965), (91.69391787862006,26.19086918160832), (91.69392377021562,26.19060517554674), (91.69405558550459,26.19026870336274), (91.69412551333575,26.18996338523236)]
for (i,j) in coordinates:
    pos = latlng_to_screen_xy(i,j)
    point = {pos["x"], pos["y"]}
    print(point)
'''pos = latlng_to_screen_xy(26.18975333575588, 91.69428490362668)
point = {"x": pos["x"], "y": pos["y"]}
print(point)'''