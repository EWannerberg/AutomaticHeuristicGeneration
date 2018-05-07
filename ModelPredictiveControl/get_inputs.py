__author__ = 'Erik Wannerberg'


def get_inputs_from_file(filename=""):
    """
    Get the specified JSON input file's contents as a dict

    :param filename: name of the JSON input file
    :return: json/python dictionary
    :rtype: dict
    """
    import json

    with open(filename) as input_text:
        json_obj = json.load(input_text)
        return json_obj


def dump_json_dict(out_dict=None, filename=""):
    if filename != "" and out_dict is not None:
        import json

        out_file = open(filename, "w")
        json.dump(out_dict, out_file, indent=4, separators=(',', ': '))
    else:
        print("Didn't specify a file to dump any json dict to!")


def save_lists_to_file(filename, elev_list, dist_list):
    """
    Convention for saving the elev arrays to file.
    :param filename:
    :param elev_list:
    :param dist_list:
    :return:
    """
    import numpy as np

    np.save(file=filename,arr=np.array([elev_list, dist_list]))


def get_lists_from_file(filename):
    """
    Convention for loading according to saving function above.
    :param filename:
    :return: elev_list, dist_list
    """
    import numpy as np

    elev_arrays = np.load(file=filename)

    return list(elev_arrays[0]), list(elev_arrays[1])


def write_output_csv(filename, **kwargs):
    """
    Writing a csv file of the keyword-specified lists.
    :param filename: Name of csv-file
    :type filename: str
    :param intermediate: Toggles filename as intermediate without timestamp
    :param kwargs: keyword-specified list-equivalents of outputs (example a=[1,2,3], b=[4,5,6])
    :return:
    """
    import csv
    import time

    intermediate = kwargs.pop("intermediate", False)

    keys = sorted(kwargs.keys())
    num_vars = len(keys)

    if intermediate:
        full_filename = filename + "_interm"
    else:
        dot_index = filename.rfind('.')
        if dot_index != -1:
            full_filename = (filename[:dot_index]
                             + time.strftime("%Y-%m-%d-%H.%M.%S")
                             + filename[dot_index:])
        else:
            full_filename = filename + time.strftime("%Y-%m-%d-%H.%M.%S")

    # add current time to filename as an identifier
    with open(full_filename, 'w', newline='') as csvfile:

        writer = csv.writer(csvfile)

        # write header
        writer.writerow(keys)

        num_entries = len(kwargs[keys[0]])
        for i in range(num_entries):
            writer.writerow(kwargs[keys[j]][i] for j in range(num_vars))


def read_output_csv(filename):
    import csv

    with open(filename, 'r', newline='') as csvfile:
        reader = csv.reader(csvfile)

        # read header
        keys = next(reader)
        num_keys = len(keys)

        result_dict = dict([(key, []) for key in keys])

        # subject to optimization using e.g. numpy arrays and funcs
        for row in reader:
            for i in range(num_keys):
                result_dict[keys[i]].append(float(row[i]))

        return result_dict


def plot_csv_file(filename, input_file=None, plot_on_x_axis=None):
    import matplotlib.pyplot as plt
    import numpy as np

    if input_file:
        input_file_dict = get_inputs_from_file(input_file)

        v_max = input_file_dict["v_max"]
        v_min = input_file_dict["v_min"]
        force_display_scale = input_file_dict["force_display_scale"]
    else:
        v_max = v_min = force_display_scale = None

    result_dict = read_output_csv(filename)

    timepoint_history = result_dict["timepoint_history"]
    x_history = result_dict["x_history"]
    v_history = result_dict["v_history"]
    acc_history = result_dict["acc_history"]
    motor_force_history = result_dict["motor_force_history"]
    braking_force_history = result_dict["braking_force_history"]
    Temp_history = result_dict["Temp_history"]
    height_history = result_dict["height_history"]
    slope_history = result_dict["slope_history"]
    cost_history = result_dict.get("cost_history", None)

    history_x_axis = []
    if plot_on_x_axis or plot_on_x_axis is None:
        x_label_text = "Distance"
        history_x_axis = x_history
    else:
        x_label_text = "Time"
        history_x_axis = timepoint_history

    plt.figure()

    line, = plt.plot(history_x_axis, height_history, color='k')
    line.set_label('Height')
    line, = plt.plot(history_x_axis, v_history, color='g')
    line.set_label('Velocity')

    if v_max:
        plt.plot([history_x_axis[0], history_x_axis[-1]], [v_max, v_max],
                 color='g', linestyle='dashed')
    if v_min:
        plt.plot([history_x_axis[0], history_x_axis[-1]], [v_min, v_min],
                 color='g', linestyle='dashed')
    line, = plt.plot(history_x_axis, acc_history, color='b')
    line.set_label('Acceleration')
    line, = plt.plot(history_x_axis, Temp_history, color='b')
    line.set_label('Temperature')
    if force_display_scale:
        line, = plt.plot(history_x_axis,
                         np.array(motor_force_history) * force_display_scale,
                         color='r')
        line.set_label('Motor/braking force * ' + str(force_display_scale))
        plt.plot(history_x_axis, np.array(braking_force_history) * force_display_scale,
                 color='r')
    else:
        line, = plt.plot(history_x_axis, motor_force_history, color='r')
        line.set_label('Motor/braking force (possibly scaled)')
        plt.plot(history_x_axis, braking_force_history, color='r')
    if cost_history:
        line, = plt.plot(history_x_axis, cost_history, color='y')
        line.set_label('Instantaneous cost/time value')

    plt.xlabel(x_label_text)
    plt.ylabel("Variable value")
    plt.xlim([history_x_axis[0], history_x_axis[-1]])
    plt.legend(loc=3)

    plt.show()


def generate_input_files(elevation_folder_path, template_input_file_path):
    """
    Generate a bunch of input JSON files from a template and a bunch of elevation files
    :param elevation_folder_path: Path to folder with elevation files, also output folder
    :type elevation_folder_path: str or PathLike
    :param template_input_file_path: Path to input JSON template
    :type template_input_file_path: str or PathLike
    :return:
    """
    import pathlib
    json_dict = get_inputs_from_file(template_input_file_path)

    path_to_match = pathlib.Path(elevation_folder_path)

    for heightfile in path_to_match.glob("*.npy"):
        dot_index = str(heightfile).rfind('.')
        filename_base = str(heightfile)[:dot_index]
        opt_output_filename = filename_base + ".out"
        opt_input_filename = filename_base + ".json"

        localdict = json_dict.copy()

        localdict["output_file"] = opt_output_filename
        localdict["elevation_file"] = str(heightfile)

        dump_json_dict(out_dict=localdict, filename=opt_input_filename)

def take_away_suffix(config_folder_path, suffix):


    import pathlib
    import os
    path_to_match = pathlib.Path(config_folder_path)

    for config_file in path_to_match.glob(suffix):
        filename = str(config_file)
        suffix_index = str(config_file).rfind('.')


def seed_out_too_steep(config_folder_path, max_gradient):
    """
    Renames .json files for height files that have too steep gradients for driving.

    Due to tunnels in mountatin roads, sometimes the surface elevation at a road does
    not correspond to the actual road elevation. Therefore, the heightmap may be that
    of a road trying to climb an _actual_ mountain.

    This is undesired behaviour (well, unless we have a part car/part mounatin goat)
    and these heighmaps should be seeded out. This function renames the .json files
    with too steep height map files.

    :param config_folder_path: Folder path name for .json config files.
    :param max_gradient: maximum gradient allowed in a path.

    """

    import pathlib
    import os

    path_to_match = pathlib.Path(config_folder_path)

    for config_file in path_to_match.glob("*.json"):
        config_filename = str(config_file)
        json_config = get_inputs_from_file(config_filename)
        try:
            elevation_file = json_config["elevation_file"]
            elev_list, dist_list = get_lists_from_file(elevation_file)
            elev_list, dist_list = average_double_pts(elev_list, dist_list)
            if exceeds_max_gradient(elev_list, dist_list, max_gradient):
                print("Too steep slope in {}, renaming".format(config_file))
                os.rename(config_filename, config_filename + "_too_steep_slope")
            else:
                print("{} is ok".format(config_file))

        except (AttributeError, KeyError) as e:
            print('No attribute "elevation file" for {}. Skipping with exception:\n {}'.format(config_file, e))
        except (FileNotFoundError) as e:
            print("File not found, exception:\n {}".format(e))
        except Exception as e:
            print("some odd exception :( \n {}".format(e))


def exceeds_max_gradient(elev_list, dist_list, max_gradient, minimum_point_distance=20):
    import numpy as np

    elev_array = np.array(elev_list)
    dist_array = np.array(dist_list)


    interpt_distance_array = dist_array[1:] - dist_array[:-1]
    abs_slope_array = np.abs((elev_array[1:] - elev_array[:-1])/(dist_array[1:] - dist_array[:-1]))

    # minimum_point_distance ensures that doubly-sampled points are caught and ignored
    return (np.logical_and(abs_slope_array >= max_gradient,
                           interpt_distance_array >= minimum_point_distance)).any()


def average_double_pts(elev_list, dist_list, minimum_point_distance=0.5):
    """
    Create new lists were previously doubled points in dist list removed and averaged in elev_list
    :param elev_list: elevation list
    :type elev_list: list[float]
    :param dist_list: distance list
    :type dist_list: list[float]
    :param minimum_point_distance:
    :return: new_dist_list, new_elev_list
    :rtype: list[float], list[float]
    """
    import numpy as np

    dist_array = np.array(dist_list)

    diffs = dist_array[1:] - dist_array[:-1]
    bad_pts = diffs < minimum_point_distance
    bad_indices = [i for i in range(len(bad_pts)) if bad_pts[i]]

    new_elev_array = np.array(elev_list)

    for i in bad_indices:
        mean_elevation = (elev_list[i] + elev_list[i+1])/2
        new_elev_array[i] = mean_elevation
        new_elev_array[i+1] = mean_elevation

    new_dist_array = dist_array[0:-1]
    new_dist_array = new_dist_array[np.logical_not(bad_pts)]

    new_elev_array = np.array(new_elev_array[0:-1], copy=True)
    new_elev_array = new_elev_array[np.logical_not(bad_pts)]

    new_dist_list = list(new_dist_array)
    new_dist_list.append(dist_list[-1])
    new_elev_list = list(new_elev_array)
    new_elev_list.append(elev_list[-1])

    return new_elev_list, new_dist_list


def generate_heightfile(filename, wlen_components, mag_components,
                        function=None, hfile_len=None, num_datapts=None):
    from collections import Iterable
    import numpy as np

    if not isinstance(wlen_components, Iterable):
        wlen_components = np.array([wlen_components])
    if not isinstance(mag_components, Iterable):
        mag_components = np.array([mag_components])
    if function is None:
        function = np.cos

    if hfile_len is None:
        hfile_len = max(wlen_components) * 10
    if num_datapts is None:
        num_datapts = hfile_len/min(wlen_components) * len(wlen_components) * 40

    assert len(wlen_components) == len(mag_components)

    dist_list = np.linspace(start=0, stop=hfile_len, num=num_datapts)
    elev_list = np.zeros_like(dist_list)
    for i in range(len(wlen_components)):
        elev_list += mag_components[i] * function(2*np.pi/wlen_components[i] * dist_list)

    save_lists_to_file(filename, elev_list, dist_list)

