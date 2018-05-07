__author__ = 'erik'


from get_key import get_elevation_key
import googlemaps
import googlemaps.convert as con


# inspired by https://developers.google.com/maps/documentation/elevation/intro


# http://maps.googleapis.com/maps/api/elevation/outputFormat?parameters

request_client = googlemaps.Client(key=get_elevation_key())


def download_height_maps(list_of_cities, resolution=10):
    import os
    num_cities = len(list_of_cities)
    for i in range(num_cities-1):
        for j in range(i+1, num_cities):
            from_city = list_of_cities[i].split(',')[0]
            to_city = list_of_cities[j].split(',')[0]

            output_filename = "{}_to_{}_res{}.npy".format(from_city, to_city, resolution)

            if not os.path.exists(output_filename): # check existence to avoid double work
                print("Retrieving heightmap from {} to {}:".format(from_city, to_city))
                _elev_list, _dist_list, _routes_object = from_a_to_b(list_of_cities[i], list_of_cities[j], resolution=resolution)
                save_lists_to_file("{}_to_{}_res{}.npy".format(from_city, to_city, resolution),
                                   _elev_list, _dist_list)
            else:
                print("Height file from {} to {} already exists, skipping".format(
                    from_city, to_city))


def from_a_to_b(source="Munich, Germany", destination="Augsburg, Germany", resolution=50):
    """
    Try to print the way from source to destination
    :param source: The starting place
    :param destination: The destination place
    :param resolution: (minimum) resolution of height results in meters
    :return:
    """
    import numpy as np

    _include_interm_endpoints = False

    _routes_object = request_client.directions(source, destination,
                                               mode="driving", units="metric")

    # can only have 512 requests, so do some splitting to get enough resolution
    split_polylines, coordinate_distances = divide_polyline(
        _routes_object[0]["overview_polyline"]["points"],
        biggest_polyline_length=512 * resolution, include_interm_endpoints=True,
        accumulate_distances=True)

    total_elev_list = []
    total_dist_list = []
    accumulated_length = 0
    for polyline in split_polylines:
        print('requesting ', polyline)
        elev_result = request_client.elevation_along_path(path=polyline, samples=512)
        _elev_list, _dist_list = lists_of_elev_results(elev_list=elev_result)
        if _include_interm_endpoints:
            total_elev_list += _elev_list
            total_dist_list += list(np.array(_dist_list) + accumulated_length)
        else:
            total_elev_list = total_elev_list[0:-1] + _elev_list
            total_dist_list = total_dist_list[0:-1] + list(np.array(_dist_list)
                                                           + accumulated_length)

        accumulated_length = total_dist_list[-1]

    return total_elev_list, total_dist_list, _routes_object


def divide_polyline(polyline_string_or_list=[""], biggest_polyline_length=None,
                    include_interm_endpoints=True, accumulate_distances=True):
    """
    Divide up an encoded polyline or lists of polylines into lists of polylines
    with specified length

    :param polyline_string_or_list: encoded polyline string or list of strings
    :type polyline_string_or_list: str or List(str)
    :param biggest_polyline_length: cut-off length for polylines
    :type biggest_polyline_length: numbers.Number
    :param include_interm_endpoints: whether to include the first point in the next
    :type include_interm_endpoints: bool
    :param accumulate_distances: whether to accumulate distances along the lines
    :type accumulate_distances: bool
    :return: list of encoded polylines with specified length (except last)
    :rtype: list of str
    """
    import numpy as np

    if isinstance(polyline_string_or_list, str):
        polyline_string_or_list = [polyline_string_or_list]
    if biggest_polyline_length is None:
        biggest_polyline_length = np.inf

    # generator over all coordinates in list
    def next_coordinate(list_of_polyline_strings):
        for _poly_string in list_of_polyline_strings:
            coordinate_obj_list = con.decode_polyline(_poly_string)
            for coord in coordinate_obj_list:
                yield coord

    all_coordinates = next_coordinate(polyline_string_or_list)
    minimum_coord_distance = 0.5
    result_polyline_lists = []
    result_coordinate_distances = []
    accumulated_distance = 0
    previous_polylines_length = 0
    current_distance_list = [accumulated_distance]
    previous_coordinate = next(all_coordinates)
    current_coordinate_list = [previous_coordinate]

    for coordinate in all_coordinates:
        reached_end_of_leg = False
        while not reached_end_of_leg:

            # ignore elevation
            next_leg_length = distance_between_coordinates(previous_coordinate,
                                                           coordinate)
            if next_leg_length < minimum_coord_distance:  # ignore points too close
                reached_end_of_leg = True  # go to next coordinate

            elif accumulated_distance + next_leg_length < biggest_polyline_length:
                # can still add to polyline
                current_coordinate_list.append(coordinate)
                accumulated_distance += next_leg_length
                current_distance_list.append(accumulated_distance)
                previous_coordinate = coordinate
                reached_end_of_leg = True

            else:  # split leg into two parts and start new polyline
                required_leg_length = biggest_polyline_length - accumulated_distance
                fraction_of_road_dist = required_leg_length / next_leg_length

                # Note: this is an approximation for small distances. At 100km distance,
                # this formula might give a result that is ~270 m off.

                inbetween_coord_lat = (previous_coordinate["lat"] + fraction_of_road_dist
                                       * (coordinate["lat"] - previous_coordinate["lat"]))
                inbetween_coord_lng = (previous_coordinate["lng"] + fraction_of_road_dist
                                       * (coordinate["lng"] - previous_coordinate["lng"]))
                inbetween_coord = dict([("lat", inbetween_coord_lat),
                                        ("lng", inbetween_coord_lng)])
                accumulated_distance += fraction_of_road_dist * next_leg_length

                if include_interm_endpoints:
                    current_coordinate_list.append(inbetween_coord)

                    # should be == n * biggest_polyline_length
                    current_distance_list.append(accumulated_distance)

                # add finished polyline to lists
                result_polyline_lists.append(con.encode_polyline(current_coordinate_list))
                if accumulate_distances:
                    result_coordinate_distances.append(
                        list(np.array(current_distance_list) + previous_polylines_length))
                    previous_polylines_length += accumulated_distance
                else:
                    result_coordinate_distances.append(current_distance_list)

                current_coordinate_list = [inbetween_coord]
                previous_coordinate = inbetween_coord
                accumulated_distance = 0
                current_distance_list = [accumulated_distance]

    # add last lists
    result_polyline_lists.append(con.encode_polyline(current_coordinate_list))

    # previous_polylines_length only increases if accumulate_distances is set to True
    result_coordinate_distances.append(list(np.array(current_distance_list)
                                            + previous_polylines_length))

    return result_polyline_lists, result_coordinate_distances


def distance_between_coordinates(coordinate1=dict([("lat", 0), ("lng", 0)]),
                                 coordinate2=dict([("lat", 0), ("lng", 0)]),
                                 elev1=0, elev2=0):
    import math
    earth_radius = 6365265  # approximately in germany at 51 deg north
    coord_to_rads = math.pi / 180

    diffy = (coordinate1["lat"] - coordinate2["lat"]) * coord_to_rads * earth_radius

    # latitude lines grow shorter the further from the equator
    diffx = ((coordinate1["lng"] - coordinate2["lng"])
             * math.cos((coordinate1["lat"] + coordinate2["lat"]) / 2 * coord_to_rads)
             * coord_to_rads * earth_radius)

    elev_diff = (elev1 - elev2)

    return math.sqrt(diffx ** 2 + diffy ** 2 + elev_diff ** 2)


def lists_of_elev_results(elev_list):
    """
    Transform the googlemaps object into two lists
    :param elev_list: googlemaps results object for elevation request
    :return: elevation_list, distance_list: lists of elevation and approx. distance for the points
    """
    cumul_dist = 0
    distance_list = []
    elevation_list = []

    distance_list.append(cumul_dist)
    elevation_list.append(elev_list[0]["elevation"])

    previous_obj_coordinates = elev_list[0]["location"]

    # earth_radius = 6365265
    # https://rechneronline.de/earth-radius/ at latitude 51 deg N sea level(~Germany)

    # using small angle approximation for distances
    for position_object in elev_list[1:]:
        elevation_list.append(position_object["elevation"])
        current_obj_coordinates = position_object["location"]

        cumul_dist += distance_between_coordinates(current_obj_coordinates,
                                                   previous_obj_coordinates,
                                                   elevation_list[-1],
                                                   elevation_list[-2])
        distance_list.append(cumul_dist)

        previous_obj_coordinates = current_obj_coordinates

    return elevation_list, distance_list


def save_lists_to_file(filename, elev_list, dist_list):
    """
    Convention for saving the elev arrays to file.
    :param filename:
    :param elev_list:
    :param dist_list:
    :return:
    """
    import numpy as np

    np.save(file=filename, arr=np.array([elev_list, dist_list]))


def get_lists_from_file(filename):
    """
    Convention for loading according to saving function above.
    :param filename:
    :return: elev_list, dist_list
    """
    import numpy as np

    elev_arrays = np.load(file=filename)

    return list(elev_arrays[0]), list(elev_arrays[1])


