function device_outage_parameter!(
    container::PSI.OptimizationContainer,
    constraint_info::Vector{DeviceOutageConstraintInfo},
    T::Type{OutageCommitmentConstraint},
    devices::IS.FlattenIteratorWrapper{V},
) where {V <: PSY.ThermalGen}
    time_steps = PSI.get_time_steps(container)
    device_names = [PSY.get_name(d) for d in devices]
    varon = PSI.get_variable(container, PSI.OnVariable(), V)
    varstop = PSI.get_variable(container, PSI.StopVariable(), V)
    varstart = PSI.get_variable(container, PSI.StartVariable(), V)
    # var_outage = PSI.get_variable(container, OutageVariable(), V)
    # con_outage =
    #     PSI.add_constraints_container!(container, T(), V, device_names, time_steps, meta="outage")
    con_on = PSI.add_constraints_container!(container, T(), V, device_names, time_steps, meta="on")
    con_stop =
        PSI.add_constraints_container!(container, T(), V, device_names, time_steps, meta="stop")
    con_start =
        PSI.add_constraints_container!(container, T(), V, device_names, time_steps, meta="start")
    param = PSI.get_parameter_array(container, OutageTimeSeriesParameter(), V)
    multiplier = PSI.get_parameter_multiplier_array(container, OutageTimeSeriesParameter(), V)

    for cont in constraint_info
        name = get_component_name(cont)
        param[name, 1] =
            PJ.add_parameter(container.JuMPmodel, cont.timeseries[1])
        multiplier[name, 1] = cont.multiplier
        ic_outage = PSI.get_value(get_initial_condition(cont))
        varz = JuMP.@variable(
            container.JuMPmodel,
            base_name = "outage_z_{$(name), 1}"
        )
        vary = JuMP.@variable(
            container.JuMPmodel,
            base_name = "outage_y_{$(name), 1}"
        )
        JuMP.@constraint(container.JuMPmodel, varz <= varon[name, 1])
        JuMP.@constraint(container.JuMPmodel, vary <= varon[name, 1])

        JuMP.@constraint(container.JuMPmodel, varz <= ic_outage) # 
        JuMP.@constraint(container.JuMPmodel, vary <= param[name, 1] * multiplier[name, 1])

        JuMP.@constraint(
            container.JuMPmodel,
            varz >= ic_outage + varon[name, 1] - 1.0
        )
        JuMP.@constraint(
            container.JuMPmodel,
            vary >= param[name, 1] * multiplier[name, 1] + varon[name, 1] - 1.0
        )

        con_on[name, 1] = JuMP.@constraint(
            container.JuMPmodel,
            varon[name, 1] <= param[name, 1] * multiplier[name, 1]
        )
        con_start[name, 1] = JuMP.@constraint(
            container.JuMPmodel,
            varstart[name, 1] <= param[name, 1] * multiplier[name, 1]
        )
        con_stop[name, 1] = JuMP.@constraint(
            container.JuMPmodel,
            varstop[name, 1] >= varz - vary
        )
        # con_outage[name, 1] = JuMP.@constraint(
        #     container.JuMPmodel,
        #     var_outage[name, 1] == param[name, 1]
        # )

        for t in time_steps[2:end]
            param[name, t] =
                PJ.add_parameter(container.JuMPmodel, cont.timeseries[t])
            multiplier[name, t] = cont.multiplier
            varz = JuMP.@variable(
                container.JuMPmodel,
                base_name = "outage_z_{$(name), $(t)}"
            )
            vary = JuMP.@variable(
                container.JuMPmodel,
                base_name = "outage_y_{$(name), $(t)}"
            )
            JuMP.@constraint(container.JuMPmodel, varz <= varon[name, t])
            JuMP.@constraint(container.JuMPmodel, vary <= varon[name, t])

            JuMP.@constraint(container.JuMPmodel, varz <= param[name, t - 1] * multiplier[name, t - 1])
            JuMP.@constraint(container.JuMPmodel, vary <= param[name, t] * multiplier[name, t])

            JuMP.@constraint(
                container.JuMPmodel,
                varz >= param[name, t - 1] * multiplier[name, t - 1] + varon[name, t] - 1.0
            )
            JuMP.@constraint(
                container.JuMPmodel,
                vary >= param[name, t] * multiplier[name, t] + varon[name, t] - 1.0
            )
            con_stop[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                varstop[name, t] >= varz - vary
            )
            con_start[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                varstart[name, t] <= param[name, t] * multiplier[name, t]
            )
            con_on[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                varon[name, t] <= param[name, t] * multiplier[name, t]
            )
            # con_outage[name, t] = JuMP.@constraint(
            #     container.JuMPmodel,
            #     var_outage[name, t] == param[name, t]
            # )
        end
    end
    return
end


function device_outage_ub_parameter!(
    container::PSI.OptimizationContainer,
    constraint_info::Vector{DeviceOutageConstraintInfo},
    T::Type{OutageUpperBoundConstraint},
    devices::IS.FlattenIteratorWrapper{V},
    ::S,
) where {V <: Union{PSY.HydroGen, PSY.HydroPumpedStorage, PSY.RenewableGen, PSY.Storage, PSY.ThermalGen}, S <: PSI.VariableType}
    time_steps = PSI.get_time_steps(container)
    device_names = [PSY.get_name(d) for d in devices]
    varp = PSI.get_variable(container, S(), V)
    # var_outage = PSI.get_variable(container, OutageVariable(), V)
    constraint =
        PSI.add_constraints_container!(container, T(), V, device_names, time_steps, meta="power")
    # con_outage =
    #     PSI.add_constraints_container!(container, T(), V, device_names, time_steps, meta="outage")
    param = PSI.get_parameter_array(container, OutageTimeSeriesParameter(), V)
    multiplier = PSI.get_parameter_multiplier_array(container, OutageTimeSeriesParameter(), V)

    for cont in constraint_info, t in time_steps
        name = get_component_name(cont)
        # ?NG: do we need this line below?
        param[name, t] =
            PJ.add_parameter(container.JuMPmodel, cont.timeseries[t])
        multiplier[name, t] = cont.multiplier
        constraint[name, t] = JuMP.@constraint(
            container.JuMPmodel,
            varp[name, t] <= param[name, t] * multiplier[name, t] * PSI.M_VALUE
        )
        # con_outage[name, t] = JuMP.@constraint(
        #     container.JuMPmodel,
        #     var_outage[name, t] == param[name, t]
        # )
    end
    return
end
