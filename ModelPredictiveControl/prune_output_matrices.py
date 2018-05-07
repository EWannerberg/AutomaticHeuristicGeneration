__author__ = 'Erik Wannerberg'

from collections import namedtuple

PrioQDataPt = namedtuple('PrioQDataPt', 'value index id nearest_neighbour_index nearest_neighbour_id')


def append_file_to_input_and_prune(input_mat, filename, keep_size, max_matrix_size,
                                  timelag_steps=100, timelag_scaling=0.2,
                                  timelag_columns=slice(None), assign_id=None):
    import numpy as np

    assert isinstance(input_mat, np.ndarray)

    data = np.loadtxt(filename, dtype=np.float32, skiprows=1, delimiter=',')
    if assign_id is not None:
        data = np.concatenate((data, assign_id*np.ones((data.shape[0], 1))), axis=1)

    data_width = data.shape[1]
    zdata = data[:, timelag_columns]/np.std(data[:, timelag_columns], axis=0)
    tdata = create_timelagged_matrices(zdata, timelag_steps, timelag_scaling)

    assert (input_mat.shape[1] == data.shape[1] + tdata.shape[1])

    new_data_full = np.concatenate((data[:tdata.shape[0]], tdata), axis=1)
    combined_data = np.concatenate((input_mat, new_data_full), axis=0)
    comb_proc_data = np.concatenate((np.arange(0, combined_data.shape[0]).reshape((combined_data.shape[0], 1)),
                                    combined_data[:, data_width:]), axis=1)

    pdata = prune_matrix(comb_proc_data, num_to_keep=keep_size,
                         max_matrix_size=max_matrix_size,
                         column_dist_subset=slice(1, None))

    pdata = sort_to_usability(pdata, column_dist_subset=slice(1, None))

    result_data = combined_data[pdata[:, 0].astype(int, copy=False), :]

    return result_data


def load_and_prepare_file(filename, keep_size, max_matrix_size,
                          timelag_steps=100, timelag_scaling=0.2,
                          timelag_columns=slice(None), assign_id=None):

    import numpy as np

    data = np.loadtxt(filename, dtype=np.float32, skiprows=1, delimiter=',')

    if assign_id is not None:
        data = np.concatenate((data, assign_id*np.ones((data.shape[0], 1))), axis=1)

    zdata = data[:, timelag_columns]/np.std(data[:, timelag_columns], axis=0)
    tdata = create_timelagged_matrices(zdata, timelag_steps, timelag_scaling)
    combined_data = np.concatenate((np.arange(0, tdata.shape[0]).reshape((tdata.shape[0], 1)), tdata), axis=1)

    pdata = prune_matrix(combined_data, num_to_keep=keep_size,
                         max_matrix_size=max_matrix_size,
                         column_dist_subset=slice(1, None))

    pdata = sort_to_usability(pdata, column_dist_subset=slice(1, None))

    result_data = np.concatenate((data[pdata[:, 0].astype(int, copy=False), :], pdata[:, 1:]), axis=1)

    return result_data


def prune_matrix(input_matrix, num_to_keep, max_matrix_size,
                 column_dist_subset=slice(None)):

    import numpy as np

    num_rows = input_matrix.shape[0]


    iteration_size = max_matrix_size - num_to_keep

    current_data = input_matrix[0:max_matrix_size, :]
    current_start = min(max_matrix_size, num_rows)  # max on ~8GB ram, 10GB swap ~ 24000

    while True:
        print("processing for up to first {} elts...".format(current_start))
        current_data, _ = prune_closer_points(current_data,
                                              keep_max_number=num_to_keep,
                                              column_dist_subset=column_dist_subset)
        if current_start + iteration_size < num_rows:
            current_data = np.concatenate((current_data, input_matrix[current_start:(current_start + iteration_size), :]))
            current_start += iteration_size
        elif current_start < num_rows:
            current_data = np.concatenate((current_data, input_matrix[current_start:]))
            current_start = num_rows
        else:
            return current_data


def prune_closer_points(input_data, column_dist_subset=slice(None),
                        keep_max_number=None, ignore_distances_under=None):

    import scipy.spatial.distance as sd
    import numpy as np
    import heapq

    num_inputs = np.shape(input_data)[0]
    dist_mat = sd.squareform(sd.pdist(input_data[:, column_dist_subset],
                                      metric='sqeuclidean'))

    # find the closest neighbours to all points

    # add something big to diagonal to not see self as closest
    biggest_value = np.amax(dist_mat)
    dist_mat[np.arange(0, num_inputs), np.arange(0, num_inputs)] = biggest_value
    minimum_dist_indices = np.argmin(dist_mat, axis=1)
    minimum_dist_values = dist_mat[np.arange(0, num_inputs), minimum_dist_indices]

    if keep_max_number is None or keep_max_number > num_inputs:
        keep_max_number = 0

        if ignore_distances_under is None:
            ignore_distances_under = np.percentile(minimum_dist_values, 25,
                                                   interpolation='nearest')
    elif ignore_distances_under is None:
        ignore_distances_under = np.inf

    ignore_distances_under **= 2
    prune = np.zeros_like(minimum_dist_indices, dtype=bool)

    pqueue = [(minimum_dist_values[i], i, minimum_dist_indices[i])
              for i in range(num_inputs)]

    # turn into priority queue
    heapq.heapify(pqueue)

    # remove indices until closest two neighbours are far enough apart
    while pqueue[0][0] <= ignore_distances_under and len(pqueue) > keep_max_number:
        current_candidate = pqueue[0]
        current_index = current_candidate[1]
        # check if removed already
        if prune[current_candidate[2]]:
            # add something big where pruned already to ignore
            min_neighbour = np.argmin(dist_mat[current_index, :] + prune * biggest_value)
            new_candidate = (dist_mat[current_index, min_neighbour],
                             current_index,
                             min_neighbour)
            minimum_dist_values[current_index] = new_candidate[0]
            heapq.heapreplace(pqueue, new_candidate)
        else:
            prune[current_index] = True
            heapq.heappop(pqueue)

    # reset diagonal to 0
    dist_mat[np.arange(0, num_inputs), np.arange(0, num_inputs)] = 0

    return input_data[np.logical_not(prune), :], dist_mat[np.ix_(np.logical_not(prune), np.logical_not(prune))]


def sort_to_usability(input_data, column_dist_subset=slice(None)):
    import scipy.spatial.distance as sd
    import numpy as np
    import heapq

    num_inputs = np.shape(input_data)[0]
    dist_mat = sd.squareform(sd.pdist(input_data[:, column_dist_subset],
                                      metric='sqeuclidean'))

    # find the closest neighbours to all points
    biggest_value = np.amax(dist_mat)
    # add something big to diagonal to not see self as closest
    dist_mat[np.arange(0, num_inputs), np.arange(0, num_inputs)] = biggest_value
    minimum_dist_indices = np.argmin(dist_mat, axis=1)
    minimum_dist_values = dist_mat[np.arange(0, num_inputs), minimum_dist_indices]

    importance_order = np.zeros_like(minimum_dist_indices, dtype=int)

    pqueue = [(minimum_dist_values[i], i, minimum_dist_indices[i])
              for i in range(num_inputs)]

    # turn into priority queue
    heapq.heapify(pqueue)

    # remove indices until closest two neighbours are far enough apart
    while len(pqueue) > 1:
        current_candidate = pqueue[0]
        current_index = current_candidate[1]
        # check if removed already
        if importance_order[current_candidate[2]] != 0:
            # add something big where sorted away already to ignore
            min_neighbour = np.argmin(dist_mat[current_index, :]
                                      + (importance_order > 0) * biggest_value)
            new_candidate = (dist_mat[current_index, min_neighbour],
                             current_index,
                             min_neighbour)
            minimum_dist_values[current_index] = new_candidate[0]
            heapq.heapreplace(pqueue, new_candidate)
        else:
            importance_order[current_index] = len(pqueue) - 1
            heapq.heappop(pqueue)

    index_order = np.zeros_like(importance_order)
    index_order[importance_order] = np.arange(0, len(importance_order))

    # reset diagonal to 0
    dist_mat[np.arange(0, num_inputs), np.arange(0, num_inputs)] = 0
    return input_data[index_order, :]


def add_data_pt(data_mat, pt_data_vec, prio_q, new_pt, min_distance=0):
    import numpy as np
    import heapq

    new_pt_distancesq = np.sum(np.square(data_mat - new_pt), axis=1)

    nearest_neigh = np.argmin(new_pt_distancesq)
    nndist_sq = new_pt_distancesq[nearest_neigh]

    if nndist_sq > min_distance**2:
        while nndist_sq > prio_q[0].value:
            # add the new point
            # check that the old is removable
            remove_pt = prio_q[0]
            if pt_data_vec[remove_pt.nearest_neigh].id == remove_pt.nearest_neighbour_id:
                # all is well, remove
                nnid = pt_data_vec[nearest_neigh].id
                new_pt_data = PrioQDataPt(value=nndist_sq, index=remove_pt.index, id=remove_pt.id+1,
                                          nearest_neighbour_index=nearest_neigh,
                                          nearest_neighbour_id=nnid)
                pt_data_vec[remove_pt.index] = new_pt_data
                data_mat[remove_pt.index, :] = new_pt
                heapq.heapreplace(prio_q, new_pt_data)
                break
            else:
                # could do more efficient by keeping sorted list of nearest neighbours to skip
                # this calculation every time...
                old_pt_distancesq = np.sum(np.square(data_mat - data_mat[remove_pt.index, :]), axis=1)

                old_nearest_neigh = np.argmin(old_pt_distancesq)
                old_distsq = old_pt_distancesq[old_nearest_neigh]

                old_pt_new = remove_pt._replace(value=old_distsq,
                                                nearest_neighbour_index=old_nearest_neigh,
                                                nearest_neighbour_id=pt_data_vec[old_nearest_neigh].id)
                # reinsert point
                heapq.heapreplace(prio_q, old_pt_new)


def create_timelagged_matrices(input_data, timelag_length, last_elt_scale):
    import numpy as np

    input_array = np.array(input_data)
    if len(input_array.shape) == 1:
        input_array = input_array.reshape((len(input_array), 1))

    input_data_size = input_array.shape
    timelagged_size = (input_data_size[0] - timelag_length,
                       input_data_size[1]*(timelag_length + 1))

    result_matrix = np.zeros(timelagged_size, dtype=np.float32)

    result_matrix[:, :input_data_size[1]] = input_array[:timelagged_size[0], :]

    attenuation_alpha = np.log(last_elt_scale) / timelag_length
    for i in range(1,timelag_length+1):
        tl_var_start = input_data_size[1] * i
        result_matrix[:, tl_var_start:(tl_var_start+input_data_size[1])] = (
            input_array[i:(timelagged_size[0] + i), :] * np.exp(i * attenuation_alpha)
        )

    return result_matrix


def filter_away_extreme_slopes(input_data, slope_col, max_slope, filter_width=0):
    """
    Filter away everywhere where slope is over max_slope, plus surrounding entries
    :type input_data: numpy.ndarray
    :type slope_col: int or slice or list[int]
    :type max_slope: float
    :type filter_width: int
    :return:
    """
    import numpy as np
    indices = np.less(np.abs(input_data[:, slope_col]), max_slope)

    widened_indices = np.convolve(np.reshape(indices, len(indices)),
                                  np.ones(filter_width, dtype=bool), mode='same')

    return input_data[widened_indices, :]
