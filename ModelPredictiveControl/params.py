

class Params:
    """
    Class for reading, storing and some limited pre/post-processing of parameters.
    """

    def __init__(self, input_file):
        """
        Initialise parameters from input file, with defaults of some values.
        :param input_file: name of input json file
        """
        import get_inputs as gi
        import math

        assert isinstance(input_file, str)

        _params = gi.get_inputs_from_file(input_file)

        # load height files
        elev_list, dist_list = gi.get_lists_from_file(_params["elevation_file"])

        # clean up height files
        elev_list, dist_list = gi.average_double_pts(elev_list=elev_list,
                                                     dist_list=dist_list)

        _params["elev_list"] = elev_list
        _params["dist_list"] = dist_list
        _params["init_dist"] = max(_params.get("init_dist", 0), dist_list[0])
        _params["end_dist"] = min(_params.get("end_dist", math.inf), dist_list[-1])

        _params["episode_length"] = _params.get("episode_length", 10000)
        _params["episode_dist_init"] = _params.get("episode_dist_init", 0)
        _params["episode_dist_end"] = (_params["episode_dist_init"]
                                       + _params["episode_length"])
        
        # half-time 1 hour
        _params["cool_exponent"] = (_params["motor_heat_capac"]
                                    * math.log(2) / _params["rad_cool_half_time"])

        _params["v_bounds"]            = (_params["v_min"], _params["v_max"])
        _params["Temp_bounds"]         = (-_params["Temp_max"], _params["Temp_max"])
        _params["motor_force_bounds"]  = (0, _params["motor_max_power"])
        _params["braking_force_bounds"]= (_params["max_braking_power"], 0)

        # if only one exists, use that one
        _params["v_target"]            = _params.get("v_target"  , _params["v_init"])
        _params["v_init"]            = _params.get("v_init"      , _params["v_target"])

        _params["v_soft_min"]          = _params.get("v_soft_min", _params["v_target"])
        _params["v_soft_max"]          = _params.get("v_soft_max", (_params["v_target"]
                                                                    + _params["v_max"])/2)

        _params["ignore_fourier_terms_under"] = _params.get("ignore_fourier_terms_under",
                                                            10)

        _params["min_heightpt_dist"] = _params.get("min_heightpt_dist", 1)

        _params["plot_after_finish"] = _params.get("plot_after_finish", True)
        _params["output_file"] = _params.get("output_file", None)
        _params["max_consecutive_fails"] = _params.get("max_consecutive_fails", 15)

        _params["mforce_max_rel_chrate"] = _params.get("mforce_max_rel_chrate", 0.2)
        motor_force_max_change_rate = _params["motor_max_power"]/_params["v_target"] * _params["mforce_max_rel_chrate"]
        _params["motor_force_deriv_bounds"] = _params.get("motor_force_deriv_bounds",
                                                          (-motor_force_max_change_rate,
                                                           motor_force_max_change_rate))
        _params["motor_smooth_coef"] = _params.get("motor_smooth_coef",
                                                   (_params["motor_max_power"]
                                                    / _params["v_target"]
                                                    * _params["mforce_max_rel_chrate"]
                                                    /1000000))

        _params["height_smoothing_length"] = _params.get("height_smoothing_length", 100)
        _params["output_intermediate_after"] = _params.get("output_intermediate_after",
                                                           math.inf)

        self._spline_interpolation = None  # type: scipy.interpolate.UnivariateSpline
        self._params = _params

    def get_li_height(self, distance):
        """
        Return a linearly interpolated height from the height file.
        :param distance: distance along the road described by ths height file.
        :type distance: float
        :return: interpolated height
        :rtype: float
        """
        import bisect

        assert(self._params["init_dist"] <= distance < self._params["end_dist"])

        dist_list = self._params["dist_list"]
        elev_list = self._params["elev_list"]

        ind_pt_left = bisect.bisect_left(dist_list, distance)
        try:
            ind_pt_right = ind_pt_left + 1

            while (dist_list[ind_pt_right] - dist_list[ind_pt_left]
                   < self._params["min_heightpt_dist"]):
                ind_pt_right += 1

            return elev_list[ind_pt_left] + ((distance - dist_list[ind_pt_left])
                                             / (dist_list[ind_pt_right]
                                                - dist_list[ind_pt_left])
                                             * (elev_list[ind_pt_right]
                                                - elev_list[ind_pt_left]))
        except IndexError as e:
            print("WARNING: trying to access out-of-bounds height data index [{}] while at dist={}/{}, (lower index={}, len={})".format(ind_pt_right,distance,self._params["end_dist"],ind_pt_left,len(dist_list)))
            return elev_list[ind_pt_left]

    def get_spline_height(self, distance):
        """
        Use scipy's cubic spline to interpolate/evaluate a fitted spline.
        :param distance: distance point
        :type distance: float or Iterable(float)
        :return: (smoothened) height at point distance
        :rtype: float
        """
        import scipy.interpolate as ip
        import math

        dist_list = self._params["dist_list"]

        if len(distance) > 1:
            assert ((dist_list[0] <= distance).all()
                    and (distance <= dist_list[-1]).all())
        else:
            assert (dist_list[0] <= distance <= dist_list[-1])

        if self._spline_interpolation is None:
            # If the interpolation/fitting fails, the library will issue a warning.
            # So, incrementally increase the smoothing until the fitting succeeds.
            import warnings as war
            with war.catch_warnings(record=True) as ws:  # to catch warnings
                war.simplefilter('always')
                warnings_counter = len(ws)
                smoothing_factor = math.sqrt(len(dist_list))
                self._spline_interpolation = ip.UnivariateSpline(x=dist_list,
                                                                 y=self._params["elev_list"],
                                                                 s=smoothing_factor)
                interpolation_gave_warning = len(ws) - warnings_counter  # should be 1 if a warning was caught

                while interpolation_gave_warning > 0:
                    warnings_counter = len(ws)
                    print("Spline construction failed ({} time(s)), raising smoothing factor".format(warnings_counter))
                    if smoothing_factor > 4 * len(dist_list):
                        raise Exception("Constructing height curve spline doesn't work smoothly, check data")

                    smoothing_factor *= 1.2
                    self._spline_interpolation = ip.UnivariateSpline(x=dist_list,
                                                                     y=self._params["elev_list"],
                                                                     s=smoothing_factor)
                    interpolation_gave_warning = len(ws) - warnings_counter

        return self._spline_interpolation(distance)

    def get_gaussmooth_height(self, distances, smoothlen):
        """
        Return a height smoothed with a gaussian.
        :param distance: distance point
        :type distance: float or Iterable(float)
        :param smoothlen: smoothing length
        :type smoothlen: float
        :return: (smoothened) height at point distance
        :rtype: float
        """
        import numpy as np
        num_points = len(distances)
        dx = (distances[-1] - distances[0]) / num_points
        num_smoothpts_oneside = int((2 * smoothlen) // dx)  # type: int

        smoothpts_before = np.linspace(-dx * num_smoothpts_oneside + distances[0],
                                       distances[0],
                                       num=num_smoothpts_oneside, endpoint=False)
        smoothpts_after  = np.linspace(distances[-1] + dx,
                                       distances[-1] + dx * num_smoothpts_oneside,
                                       num=num_smoothpts_oneside, endpoint=True)

        allpts = np.concatenate((smoothpts_before, distances, smoothpts_after))
        firstind_left = 0
        lastind_right = len(allpts)
        dist_list = self._params["dist_list"]

        num_weights = 2 * num_smoothpts_oneside + 1
        gaussian_weights = np.exp(-np.power(np.linspace(-2, 2, num=num_weights, endpoint=True), 2))
        gaussian_weights = gaussian_weights / sum(gaussian_weights)  # normalize

        while allpts[firstind_left] < dist_list[0]:
            firstind_left += 1

        while allpts[(lastind_right - 1)] >= dist_list[-1]:
            lastind_right -= 1

        heights = self.get_spline_height(allpts[firstind_left:lastind_right])
        last_normal_index = lastind_right - num_weights - firstind_left
        smoothed_heights = np.zeros(num_points)

        # split up in three ranges depending on where we are
        for i in range(0, firstind_left):
            these_weights = gaussian_weights[(firstind_left-i):]
            smoothed_heights[i] = (np.sum(np.multiply(these_weights,
                                                      heights[:len(these_weights)]))
                                   / np.sum(these_weights))

        for i in range(firstind_left, last_normal_index + 1):
            smoothed_heights[i] = np.sum(
                np.multiply(gaussian_weights,
                            heights[i-firstind_left:(i-firstind_left+num_weights)]))

        for i in range(last_normal_index + 1, num_points):
            these_weights = gaussian_weights[:num_weights - (i-last_normal_index)]
            smoothed_heights[i] = (np.sum(np.multiply(these_weights,
                                                      heights[i:i+len(these_weights)]))
                                   / np.sum(these_weights))

        return smoothed_heights

    def __getitem__(self, key):
        """
        Default implementation for the []-operator: getting from the dict
        :param key: the dict key
        :return: value in parameters
        """
        return self._params[key]
