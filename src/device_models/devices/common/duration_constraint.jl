function device_duration_look_ahead_outage!(
    container::PSI.OptimizationContainer,
    T::Type{OutageTimeConstraint},
    constraint_info::Vector{DeviceDurationConstraintInfo},
    devices::IS.FlattenIteratorWrapper{V},
) where {V <: PSY.ThermalGen}
    time_steps = PSI.get_time_steps(container)
    device_names = [PSY.get_name(d) for d in devices]
    varon = PSI.get_variable(container, PSI.OnVariable(), V)
    varstop = PSI.get_variable(container, PSI.StopVariable(), V)
    varstart = PSI.get_variable(container, PSI.StartVariable(), V)
    
    con_up =
        PSI.add_constraints_container!(container, T(), V, device_names, time_steps, meta="up")
    con_down =
        PSI.add_constraints_container!(container, T(), V, device_names, time_steps, meta="dn")

    for t in time_steps
        for cont in constraint_info
            name = cont.name
            # Minimum Up-time Constraint
            expr_up =
                JuMP.GenericAffExpr{Float64, JuMP.VariableRef}(0)
            for i in (t - cont.duration_data.up + 1):t
                if i in time_steps
                    i = Int64(i)
                    JuMP.add_to_expression!(expr_up, varon[name, i])
                end
            end
            if t <= cont.duration_data.up
                expr_up += first(cont.initial_duration).value
            end
            con_up[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                varstop[name, t] * cont.duration_data.up <=
                expr_up + (1 - cont.timeseries[t]) * cont.duration_data.up
            )

            # Minimum Down-time Constraint
            expr_dn =
                JuMP.GenericAffExpr{Float64, JuMP.VariableRef}(0)
            for i in (t - cont.duration_data.down + 1):t
                if i in time_steps
                    i = Int64(i)
                    JuMP.add_to_expression!(expr_dn, (1 - varon[name, i]))
                end
            end
            if t <= cont.duration_data.down
                expr_dn += last(cont.initial_duration).value
            end
            if t == 1
                expr_dn += (1 - cont.initial_outage.value) * cont.duration_data.down
            else
                expr_dn += (1 - cont.timeseries[t - 1]) * cont.duration_data.down
            end
            con_down[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                varstart[name, t] * cont.duration_data.down <=
                expr_dn + (1 - cont.timeseries[t]) * cont.duration_data.down
            )
        end
    end
    return
end


function device_duration_parameters_outage!(
    container::PSI.OptimizationContainer,
    T::Type{OutageTimeConstraint},
    constraint_info::Vector{DeviceDurationConstraintInfo},
    devices::IS.FlattenIteratorWrapper{V},
) where {V <: PSY.ThermalGen}
    time_steps = PSI.get_time_steps(container)
    device_names = [PSY.get_name(d) for d in devices]
    varon = PSI.get_variable(container, PSI.OnVariable(), V)
    varstop = PSI.get_variable(container, PSI.StopVariable(), V)
    varstart = PSI.get_variable(container, PSI.StartVariable(), V)

    con_up =
        PSI.add_constraints_container!(container, T(), V, device_names, time_steps, meta="up")
    con_down =
        PSI.add_constraints_container!(container, T(), V, device_names, time_steps, meta="dn")
    param = PSI.get_parameter_array(container, OutageTimeSeriesParameter(), V)
    multiplier = PSI.get_parameter_multiplier_array(container, OutageTimeSeriesParameter(), V)

    for t in time_steps
        for cont in constraint_info
            name = get_component_name(cont)

            # Minimum Up-time Constraint
            expr_up =
                JuMP.GenericAffExpr{Float64, JuMP.VariableRef}(0)
            for i in (t - cont.duration_data.up + 1):t
                if i in time_steps
                    i = Int64(i)
                    JuMP.add_to_expression!(expr_up, varon[name, i])
                end
            end
            if t <= cont.duration_data.up
                expr_up += cont.initial_duration[1].value
            end

            con_up[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                varstop[name, t] * cont.duration_data.up <=
                expr_up +
                (1 * cont.duration_data.up - param[name, t] * multiplier[name, t] * cont.duration_data.up)
            )

            # Minimum Down-time Constraint
            expr_dn =
                JuMP.GenericAffExpr{Float64, JuMP.VariableRef}(0)
            for i in (t - cont.duration_data.down + 1):t
                if i in time_steps
                    i = Int64(i)
                    JuMP.add_to_expression!(expr_dn, (1 - varon[name, i]))
                end
            end
            if t <= cont.duration_data.down
                expr_dn += cont.initial_duration[2].value
            end

            if t == 1
                expr_dn += (
                    cont.duration_data.down -
                    cont.initial_outage.value * cont.duration_data.down
                )
            else
                expr_dn +=
                    (cont.duration_data.down - param[name, t - 1] * multiplier[name, t - 1] * cont.duration_data.down)
            end
            con_down[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                varstart[name, t] * cont.duration_data.down <= expr_dn
            )
        end
    end
    return
end