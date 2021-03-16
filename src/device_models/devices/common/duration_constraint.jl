@doc raw"""
This formulation of the duration constraints adds over the start times looking backwards.

# LaTeX

* Minimum up-time constraint:

If ``t \leq d_{min}^{up} - d_{init}^{up}`` and ``d_{init}^{up} > 0``

`` 1 + \sum_{i=t-d_{min}^{up} + 1}^t x_i^{start} - x_t^{on} \leq 0 ``

for i in the set of time steps. Otherwise:

`` \sum_{i=t-d_{min}^{up} + 1}^t x_i^{start} - x_t^{on} \leq 0 ``

for i in the set of time steps.

* Minimum down-time constraint:

If ``t \leq d_{min}^{down} - d_{init}^{down}`` and ``d_{init}^{down} > 0``

`` 1 + \sum_{i=t-d_{min}^{down} + 1}^t x_i^{stop} + x_t^{on} \leq 1 ``

for i in the set of time steps. Otherwise:

`` \sum_{i=t-d_{min}^{down} + 1}^t x_i^{stop} + x_t^{on} \leq 1 ``

for i in the set of time steps.


# Arguments
* optimization_container::OptimizationContainer : the optimization_container model built in PowerSimulations
* duration_data::Vector{UpDown} : gives how many time steps variable needs to be up or down
* initial_duration::Matrix{InitialCondition} : gives initial conditions for up (column 1) and down (column 2)
* cons_name::Symbol : name of the constraint
* var_names::Tuple{Symbol, Symbol, Symbol}) : names of the variables
- : var_names[1] : varon
- : var_names[2] : varstart
- : var_names[3] : varstop
"""
function device_duration_retrospective_outage!(
    optimization_container::PSI.OptimizationContainer,
    constraint_info::Vector{DeviceDurationConstraintInfo},
    param_reference::Union{Nothing, PSI.UpdateRef},
    cons_name::Symbol,
    var_names::Tuple{Symbol, Symbol, Symbol},
)
    time_steps = PSI.model_time_steps(optimization_container)

    varon = PSI.get_variable(optimization_container, var_names[1])
    varstart = PSI.get_variable(optimization_container, var_names[2])
    varstop = PSI.get_variable(optimization_container, var_names[3])

    name_up = PSI.middle_rename(cons_name, PSI.PSI_NAME_DELIMITER, "up")
    name_down = PSI.middle_rename(cons_name, PSI.PSI_NAME_DELIMITER, "dn")

    initial_duration = constraint_info.initial_duration
    duration_data = constraint_info.duration_data
    set_names = [PSI.device_name(ic) for ic in initial_duration[:, 1]]
    con_up = PSI.add_cons_container!(optimization_container, name_up, set_names, time_steps)
    con_down = PSI.add_cons_container!(optimization_container, name_down, set_names, time_steps)

    container = PSI.add_param_container!(
        optimization_container,
        param_reference,
        names,
        time_steps,
    )
    multiplier = PSI.get_multiplier_array(container)
    param = PSI.get_parameter_array(container)

    for t in time_steps
        for (ix, ic) in enumerate(initial_duration[:, 1])
            name = PSI.device_name(ic)
            # Minimum Up-time Constraint
            lhs_on = JuMP.GenericAffExpr{Float64, PSI._variable_type(optimization_container)}(0)
            for i in (t - duration_data[ix].up + 1):t
                if i in time_steps
                    JuMP.add_to_expression!(lhs_on, varstart[name, i])
                end
            end
            if t <= max(0, duration_data[ix].up - ic.value) && ic.value > 0
                JuMP.add_to_expression!(lhs_on, 1)
            end
            param[name, t] = PSI.add_parameter(
                optimization_container.JuMPmodel,
                constraint_info.timeseries[t],
            )
            multiplier[name, t] = constraint_info.multiplier

            con_up[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                lhs_on - varon[name, t] <= param[name, t] * multiplier[name, t]
            )
        end

        for (ix, ic) in enumerate(initial_duration[:, 2])
            name = PSI.device_name(ic)
            # Minimum Down-time Constraint
            lhs_off =
                JuMP.GenericAffExpr{Float64, PSI._variable_type(optimization_container)}(0)
            for i in (t - duration_data[ix].down + 1):t
                if i in time_steps
                    JuMP.add_to_expression!(lhs_off, varstop[name, i])
                end
            end
            if t <= max(0, duration_data[ix].down - ic.value) && ic.value > 0
                JuMP.add_to_expression!(lhs_off, 1)
            end
            con_down[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                lhs_off + varon[name, t] <= 1.0
            )
        end
    end
    return
end

function device_duration_look_ahead_outage!(
    optimization_container::PSI.OptimizationContainer,
    constraint_info::Vector{DeviceDurationConstraintInfo},
    param_reference::Union{Nothing, PSI.UpdateRef},
    cons_name::Symbol,
    var_names::Tuple{Symbol, Symbol, Symbol},
)

end


function device_duration_parameters_outage!(
    optimization_container::PSI.OptimizationContainer,
    constraint_info::Vector{DeviceDurationConstraintInfo},
    param_reference::Union{Nothing, PSI.UpdateRef},
    cons_name::Symbol,
    var_names::Tuple{Symbol, Symbol, Symbol},
)

end
