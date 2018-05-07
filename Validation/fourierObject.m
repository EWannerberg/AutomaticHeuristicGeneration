function [ f_obj ] = fourierObject( elev_list, dist_list )
%FOURIEROBJECT Summary of this function goes here
%   Detailed explanation goes here

ignore_wlength_shorter = 10;
ignore_under = 10;



    %# try to make periodic for well-conditioned fourier transform -> eliminate jump from start to finish
gradient = (elev_list(end) - elev_list(1)) / (dist_list(end) - dist_list(1));

preproc_elev_list = elev_list - (dist_list - dist_list(1)) * gradient;

%    # print("Constructing slope fourier fun, this might take a while...")

coefs = fft(preproc_elev_list);

abs_biggest_coef = max(coefs);

original_length = max(size(elev_list));

norm_factor = original_length;

scale_factor_x = original_length/(dist_list(end)-dist_list(1));

x_offset = dist_list(1);

tot_scalefactor = 2 * pi * scale_factor_x / original_length;


n = ceil(max(size(coefs))+1)/2;
if ignore_wlength_shorter > 0
    max_freq_index = floor((dist_list(end)-dist_list(1))/ignore_wlength_shorter);
    n = min(n, max_freq_index);
end

f_obj.x_offset = x_offset;
f_obj.x_end = dist_list(end);
f_obj.norm_factor = norm_factor;
f_obj.tot_scalefactor = tot_scalefactor;
f_obj.gradient = gradient;
f_obj.n = n;
f_obj.height_offset = coefs(1)/norm_factor;

termnumbers = floor(0:(n-1));
ignore = abs(coefs) < ignore_under;
coefs(ignore) = 0;
half_coefs = coefs(1:n); % because real, only need half; the other ones are cc of first ones anyways
% sing_coefs = zeros(1,n);
% for i = 1:n
%     sing_coefs(i) = coefs(2*i-1) + coefs(2*i);
% end
% zeroes = sing_coefs == 0;
% zeroes(1) = true;
zeroes = half_coefs == 0;
zeroes(1) = true;
f_obj.termnumbers = termnumbers(~zeroes);
f_obj.coefs = 2*half_coefs(~zeroes);
f_obj.ins_x_array = (1i * f_obj.tot_scalefactor) .* (f_obj.coefs .* f_obj.termnumbers); 
f_obj.x_scale_array = 1i * f_obj.tot_scalefactor .* f_obj.termnumbers;

    function slope_value = slope_fun(the_obj, x_value)
        slope_value = real(sum(the_obj.ins_x_array .* exp(x_value * the_obj.x_scale_array)))/the_obj.norm_factor + the_obj.gradient;
    end
    function height_value = height_fun(the_obj, x_value)
        height_value = real(sum(the_obj.coefs .* exp(x_value * the_obj.x_scale_array)))/the_obj.norm_factor + the_obj.height_offset + x_value * the_obj.gradient;
    end

f_obj.slope_fun = @(x) slope_fun(f_obj,(x - f_obj.x_offset));
f_obj.height_fun = @(x) height_fun(f_obj,(x - f_obj.x_offset));

%     # tot_term_bothcoefs = lambda _complexplus, _complexminus, \
%     #                             _sin, _cos, _x: \
%     #     (- _complexminus.real - _complexplus.real) * _sin(_x) + (_complexminus.imag - _complexplus.imag) * _cos(_x)
%     sin_term = @(i_complexplus, i_complexminus, i_sin, i_x) (- i_complexminus.real - i_complexplus.real) * i_sin(i_x);
%     #+ (_complexminus.imag - _complexplus.imag) * _cos(_x)
%     cos_term = @(i_complexplus, i_complexminus, i_cos, i_x) (i_complexminus.imag - i_complexplus.imag) * i_cos(i_x);
%     # + (- _complexminus.real - _complexplus.real) * _sin(_x)

%     sin_term_single = lambda _complexplus, _sin, _x: (- _complexplus.real) * _sin(_x)
%     #+ (_complexminus.imag - _complexplus.imag) * _cos(_x)
%     cos_term_single = lambda _complexplus, _cos, _x: (- _complexplus.imag) * _cos(_x)
%     # + (- _complexminus.real - _complexplus.real) * _sin(_x)

%     # ignore too small terms and terms with too high freq

%     # indices_sin = [i for i in range(1, n) if abs(coefs[i].real + coefs[-i].real) > _ignore_under]
%     # indices_cos = [i for i in range(1, n) if abs(coefs[i].imag - coefs[-i].imag) > _ignore_under]
%     indices_sin = [i for i in range(1, n) if abs(coefs[i].real) > _ignore_under]
%     indices_cos = [i for i in range(1, n) if abs(coefs[i].imag) > _ignore_under]
%     # print(indices_cos)
%     # print(indices_sin)
%     disp('Fourier sum has {:d}/{:d} terms ({:d}s,{:d}c) over {:d}.".format(len(indices_sin) + len(indices_cos), 2 * len(coefs) - 1', len(indices_sin),len(indices_cos), ignore_under)

%     def fourier_dev(x):
%         # rewriting from complex exponentials to sins and coses since Pyomo can't handle complex
%         # return (2 *
%         #         sum((coefs[i] * 1j * math.pi * i * 2 / original_length *
%         #              env.exp(1j * math.pi * i * x * scale_factor_x * 2 / original_length)).real +
%         #             (coefs[-i] * -1j * math.pi * i * x * 2 / original_length *
%         #              env.exp(-1j * math.pi * i * x * scale_factor_x * 2 / original_length)).real for i in range(1, n) if abs(coefs[i]) > ignore_under)
%         #         ) / norm_factor
% 
%         # termlist = [
%         #     tot_scalefactor * i * tot_term_bothcoefs(coefs[i], coefs[-i], _sinfun, _cosfun, x * tot_scalefactor * i)
%         #     for i in range(1, n) if abs(coefs[i] + coefs[-i].conjugate()) > ignore_under]
%         # termlist_sin = [
%         #     tot_scalefactor * i * sin_term(coefs[i], coefs[-i], _sinfun, (x - x_offset) * tot_scalefactor * i)
%         #     for i in indices_sin]
%         # termlist_cos = [
%         #     tot_scalefactor * i * cos_term(coefs[i], coefs[-i], _cosfun, (x - x_offset) * tot_scalefactor * i)
%         #     for i in indices_cos]
%         termlist_sin = [
%             tot_scalefactor * i * sin_term_single(coefs[i], _sinfun, (x - x_offset) * tot_scalefactor * i)
%             for i in indices_sin]
%         termlist_cos = [
%             tot_scalefactor * i * cos_term_single(coefs[i], _cosfun, (x - x_offset) * tot_scalefactor * i)
%             for i in indices_cos]
% 
%         return 2 * sum(termlist_sin + termlist_cos) / norm_factor + gradient  # add gradient back
% 
%     return fourier_dev
% 
% 
% def _build_slope_constraint(_model, _dist_list, _elev_list, x_span, x_init=0, rebuild=False, _ignore_fourier_terms_under=0):
% 
%     first_index = 0
%     end_of_list = len(_dist_list)
% 
%     while _dist_list[first_index + 1] < x_init:
%         first_index += 1
% 
%     last_index = first_index + 1
% 
%     while (last_index + 1) < end_of_list and _dist_list[last_index] < x_init + x_span:
%         last_index += 1
% 
%     fourier_fun = _build_slope_fourier(_elev_list[first_index:(last_index+1)], _dist_list[first_index:(last_index+1)],
%                                        env.sin, env.cos, _ignore_under=_ignore_fourier_terms_under)
%     print("Adding slope constraint, this will take a while...")
%     if rebuild:
%         _model.del_component("slope_constraint")
% 
%         # reinitialize slope values to reflect new function
%         local_fourier_fun = _build_slope_fourier(_elev_list[first_index:(last_index+1)], _dist_list[first_index:(last_index+1)],
%                                        np.sin, np.cos, _ignore_under=_ignore_fourier_terms_under)
%         for _time in _model.t:
%             _model.slope_var[_time] = local_fourier_fun(env.value(_model.x[_time]))
% 
%     _model.slope_constraint = env.Constraint(_model.t, rule=lambda _mod, _time: _mod.slope_var[_time] == fourier_fun(_mod.x[_time]))

end

