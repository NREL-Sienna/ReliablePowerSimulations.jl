function device_duration_look_ahead_outage!(
    container::PSI.OptimizationContainer,
    duration_data::Vector{UpDown},
    initial_duration::Matrix{InitialCondition},
    cons_type::OutageTimeConstraint,
    ::Type{T},
) where {T <: PSY.ThermalGen}
    time_steps = PSI.get_time_steps(container)
    varon = PSI.get_variable(container, PSI.OnVariable(), V)
    varstop = PSI.get_variable(container, PSI.StopVariable(), V)
    varstart = PSI.get_variable(container, PSI.StartVariable(), V)
    
    set_names = [get_component_name(ic) for ic in initial_duration[:, 1]]
    con_up = add_constraints_container!(
        container,
        cons_type,
        T,
        set_names,
        time_steps,
        meta="up",
    )
    con_down = add_constraints_container!(
        container,
        cons_type,
        T,
        set_names,
        time_steps,
        meta="dn",
    )

    param = PSI.get_parameter_array(container, OutageTimeSeriesParameter(), V)
    multiplier = PSI.get_parameter_multiplier_array(container, OutageTimeSeriesParameter(), V)


    for t in time_steps
        for (ix, ic) in enumerate(initial_duration[:, 1])
            name = get_component_name(ic)
            # Minimum Up-time Constraint
            expr_up =
                JuMP.GenericAffExpr{Float64, JuMP.VariableRef}(0)
            for i in (t - duration_data.up + 1):t
                if i in time_steps
                    i = Int64(i)
                    JuMP.add_to_expression!(expr_up, varon[name, i])
                end
            end
            if t <= duration_data.up
                expr_up += get_value(ic)
            end
            con_up[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                varstop[name, t] * duration_data.up <=
                expr_up + (1 - param[name, t]) * duration_data.up
            )

            # Minimum Down-time Constraint
            expr_dn =
                JuMP.GenericAffExpr{Float64, JuMP.VariableRef}(0)
            for i in (t - duration_data.down + 1):t
                if i in time_steps
                    i = Int64(i)
                    JuMP.add_to_expression!(expr_dn, (1 - varon[name, i]))
                end
            end
            if t <= duration_data.down
                expr_dn += get_value(initial_duration[ix, 2])
            end
            if t == 1
                expr_dn += (1 - get_value(initial_duration[ix, 3])) * duration_data.down
            else
                expr_dn += (1 - param[name, t - 1]) * duration_data.down
            end
            con_down[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                varstart[name, t] * duration_data.down <=
                expr_dn + (1 - param[name, t]) * duration_data.down
            )
        end
    end
    return
end


function device_duration_parameters_outage!(
    container::PSI.OptimizationContainer,
    duration_data::Vector{UpDown},
    initial_duration::Matrix{InitialCondition},
    cons_type::OutageTimeConstraint,
    ::Type{T},
) where {V <: PSY.ThermalGen}
    time_steps = PSI.get_time_steps(container)
    varon = PSI.get_variable(container, PSI.OnVariable(), V)
    varstop = PSI.get_variable(container, PSI.StopVariable(), V)
    varstart = PSI.get_variable(container, PSI.StartVariable(), V)

    set_names = [get_component_name(ic) for ic in initial_duration[:, 1]]
    con_up = add_constraints_container!(
        container,
        cons_type,
        T,
        set_names,
        time_steps,
        meta="up",
    )
    con_down = add_constraints_container!(
        container,
        cons_type,
        T,
        set_names,
        time_steps,
        meta="dn",
    )
    param = PSI.get_parameter_array(container, OutageTimeSeriesParameter(), V)
    multiplier = PSI.get_parameter_multiplier_array(container, OutageTimeSeriesParameter(), V)

    for t in time_steps
        for (ix, ic) in enumerate(initial_duration[:, 1])
            name = get_component_name(ic)

            # Minimum Up-time Constraint
            expr_up =
                JuMP.GenericAffExpr{Float64, JuMP.VariableRef}(0)
            for i in (t - duration_data.up + 1):t
                if i in time_steps
                    i = Int64(i)
                    JuMP.add_to_expression!(expr_up, varon[name, i])
                end
            end
            if t <= duration_data.up
                expr_up += get_value(ic)
            end

            con_up[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                varstop[name, t] * duration_data.up <=
                expr_up +
                (1 * duration_data.up - param[name, t] * multiplier[name, t] * duration_data.up)
            )

            # Minimum Down-time Constraint
            expr_dn =
                JuMP.GenericAffExpr{Float64, JuMP.VariableRef}(0)
            for i in (t - duration_data.down + 1):t
                if i in time_steps
                    i = Int64(i)
                    JuMP.add_to_expression!(expr_dn, (1 - varon[name, i]))
                end
            end
            if t <= duration_data.down
                expr_dn += get_value(initial_duration[ix, 2])
            end

            if t == 1
                expr_dn += (
                    duration_data.down -
                    get_value(initial_duration[ix, 3]) * duration_data.down
                )
            else
                expr_dn +=
                    (duration_data.down - param[name, t - 1] * multiplier[name, t - 1] * duration_data.down)
            end
            con_down[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                varstart[name, t] * duration_data.down <= expr_dn
            )
        end
    end
    return
end
