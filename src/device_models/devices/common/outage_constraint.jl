function device_outage!(
    optimization_container::PSI.OptimizationContainer,
    constraint_info::Vector{DeviceOutageConstraintInfo},
    cons_name::Symbol,
    var_names::Tuple{Symbol, Symbol},
)
    time_steps = PSI.model_time_steps(optimization_container)
    varstart = PSI.get_variable(optimization_container, var_names[1])
    varstop = PSI.get_variable(optimization_container, var_names[2])

    name_start = PSI.middle_rename(cons_name, PSI.PSI_NAME_DELIMITER, "start")
    name_stop = PSI.middle_rename(cons_name, PSI.PSI_NAME_DELIMITER, "stop")

    set_names = [PSI.get_component_name(ic) for ic in constraint_info]
    con_start =
        PSI.add_cons_container!(optimization_container, name_start, set_names, time_steps)
    con_stop =
        PSI.add_cons_container!(optimization_container, name_stop, set_names, time_steps)

    for cont in constraint_info
        name = PSI.get_component_name(cont)
        con_start[name, 1] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            varstart[name, 1] >= cont.initial_condition.value - cont.timeseries[1]
        )
        con_stop[name, 1] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            varstop[name, 1] >= cont.timeseries[1] - cont.initial_condition.value
        )

        for t in time_steps[2:end]
            con_start[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                varstart[name, t] >= cont.timeseries[t - 1] - cont.timeseries[t]
            )
            con_stop[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                varstop[name, t] >= cont.timeseries[t] - cont.timeseries[t - 1]
            )
        end
    end
    return
end

function device_outage_parameter!(
    optimization_container::PSI.OptimizationContainer,
    constraint_info::Vector{DeviceOutageConstraintInfo},
    cons_name::Symbol,
    var_names::Tuple{Symbol, Symbol},
    param_reference::PSI.UpdateRef,
)
    time_steps = PSI.model_time_steps(optimization_container)
    varstart = PSI.get_variable(optimization_container, var_names[1])
    varstop = PSI.get_variable(optimization_container, var_names[2])

    name_start = PSI.middle_rename(cons_name, PSI.PSI_NAME_DELIMITER, "start")
    name_stop = PSI.middle_rename(cons_name, PSI.PSI_NAME_DELIMITER, "stop")

    set_names = [PSI.get_component_name(ic) for ic in constraint_info]
    con_start =
        PSI.add_cons_container!(optimization_container, name_start, set_names, time_steps)
    con_stop =
        PSI.add_cons_container!(optimization_container, name_stop, set_names, time_steps)

    container_outage = PSI.add_param_container!(
        optimization_container,
        param_reference,
        set_names,
        time_steps,
    )
    param = PSI.get_parameter_array(container_outage)
    multiplier = PSI.get_multiplier_array(container_outage)

    for cont in constraint_info
        name = PSI.get_component_name(cont)
        param[name, 1] =
            PJ.add_parameter(optimization_container.JuMPmodel, cont.timeseries[1])
        multiplier[name, 1] = cont.multiplier

        con_start[name, 1] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            varstart[name, 1] >= cont.initial_condition.value - param[name, 1]
        )
        con_stop[name, 1] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            varstop[name, 1] >= param[name, 1] - cont.initial_condition.value
        )

        for t in time_steps[2:end]
            param[name, t] =
                PJ.add_parameter(optimization_container.JuMPmodel, cont.timeseries[t])
            multiplier[name, t] = cont.multiplier

            con_start[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                varstart[name, t] >= param[name, t - 1] - param[name, t]
            )
            con_stop[name, t] = JuMP.@constraint(
                optimization_container.JuMPmodel,
                varstop[name, t] >= param[name, t] - param[name, t - 1]
            )
        end
    end
    return
end

function device_outage_ub!(
    optimization_container::PSI.OptimizationContainer,
    constraint_info::Vector{DeviceOutageConstraintInfo},
    cons_name::Symbol,
    var_name::Symbol,
)
    time_steps = PSI.model_time_steps(optimization_container)
    varp = PSI.get_variable(optimization_container, var_name)

    set_names = [PSI.get_component_name(ic) for ic in constraint_info]
    constraint =
        PSI.add_cons_container!(optimization_container, cons_name, set_names, time_steps)

    for cont in constraint_info, t in time_steps
        name = PSI.get_component_name(cont)
        constraint[name, t] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            varp[name, t] <= (1 - cont.timeseries[t]) * PSI.M_VALUE
        )
    end
    return
end

function device_outage_ub_parameter!(
    optimization_container::PSI.OptimizationContainer,
    constraint_info::Vector{DeviceOutageConstraintInfo},
    cons_name::Symbol,
    var_name::Symbol,
    param_reference::PSI.UpdateRef,
)
    time_steps = PSI.model_time_steps(optimization_container)
    varp = PSI.get_variable(optimization_container, var_name)

    set_names = [PSI.get_component_name(ic) for ic in constraint_info]
    constraint =
        PSI.add_cons_container!(optimization_container, cons_name, set_names, time_steps)

    container_outage = PSI.add_param_container!(
        optimization_container,
        param_reference,
        set_names,
        time_steps,
    )
    param = PSI.get_parameter_array(container_outage)
    multiplier = PSI.get_multiplier_array(container_outage)

    for cont in constraint_info, t in time_steps
        name = PSI.get_component_name(cont)
        param[name, t] =
            PJ.add_parameter(optimization_container.JuMPmodel, cont.timeseries[t])
        multiplier[name, t] = cont.multiplier
        constraint[name, t] = JuMP.@constraint(
            optimization_container.JuMPmodel,
            varp[name, t] <= (1 - param[name, t]) * PSI.M_VALUE
        )
    end
    return
end
