function device_duration_look_ahead_outage!(
    container::PSI.OptimizationContainer,
    duration_data::Vector{PSI.UpDown},
    initial_duration::Matrix{PSI.InitialCondition},
    cons_type::OutageTimeConstraint,
    ::Type{T},
) where {T <: PSY.ThermalGen}
    time_steps = PSI.get_time_steps(container)
    varon = PSI.get_variable(container, PSI.OnVariable(), T)
    varstop = PSI.get_variable(container, PSI.StopVariable(), T)
    varstart = PSI.get_variable(container, PSI.StartVariable(), T)

    set_names = [PSI.get_component_name(ic) for ic in initial_duration[:, 1]]
    con_up = PSI.add_constraints_container!(
        container,
        cons_type,
        T,
        set_names,
        time_steps,
        meta = "up",
    )
    con_down = PSI.add_constraints_container!(
        container,
        cons_type,
        T,
        set_names,
        time_steps,
        meta = "dn",
    )

    param = PSI.get_parameter_array(container, OutageTimeSeriesParameter(), T)
    multiplier =
        PSI.get_parameter_multiplier_array(container, OutageTimeSeriesParameter(), T)

    for t in time_steps
        for (ix, ic) in enumerate(initial_duration[:, 1])
            name = PSI.get_component_name(ic)
            # Minimum Up-time Constraint
            expr_up = JuMP.GenericAffExpr{Float64, JuMP.VariableRef}(0)
            for i in (t - duration_data[ix].up + 1):t
                if i in time_steps
                    i = Int64(i)
                    JuMP.add_to_expression!(expr_up, varon[name, i])
                end
            end
            if t <= duration_data[ix].up
                expr_up += PSI.get_value(ic)
            end
            con_up[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                varstop[name, t] * duration_data[ix].up <=
                expr_up + (1 - param[name, t]) * duration_data[ix].up
            )

            # Minimum Down-time Constraint
            expr_dn = JuMP.GenericAffExpr{Float64, JuMP.VariableRef}(0)
            for i in (t - duration_data[ix].down + 1):t
                if i in time_steps
                    i = Int64(i)
                    JuMP.add_to_expression!(expr_dn, (1 - varon[name, i]))
                end
            end
            if t <= duration_data[ix].down
                expr_dn += PSI.get_value(initial_duration[ix, 2])
            end
            if t == 1
                expr_dn += (1 - PSI.get_value(initial_duration[ix, 3])) * duration_data[ix].down
            else
                expr_dn += (1 - param[name, t - 1]) * duration_data[ix].down
            end
            con_down[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                varstart[name, t] * duration_data[ix].down <=
                expr_dn + (1 - param[name, t]) * duration_data[ix].down
            )
        end
    end
    return
end

function device_duration_parameters_outage!(
    container::PSI.OptimizationContainer,
    duration_data::Vector{PSI.UpDown},
    initial_duration::Matrix{PSI.InitialCondition},
    cons_type::OutageTimeConstraint,
    ::Type{T},
) where {T <: PSY.ThermalGen}
    time_steps = PSI.get_time_steps(container)
    varon = PSI.get_variable(container, PSI.OnVariable(), T)
    varstop = PSI.get_variable(container, PSI.StopVariable(), T)
    varstart = PSI.get_variable(container, PSI.StartVariable(), T)

    set_names = [PSI.get_component_name(ic) for ic in initial_duration[:, 1]]
    con_up = PSI.add_constraints_container!(
        container,
        cons_type,
        T,
        set_names,
        time_steps,
        meta = "up",
    )
    con_down = PSI.add_constraints_container!(
        container,
        cons_type,
        T,
        set_names,
        time_steps,
        meta = "dn",
    )
    param = PSI.get_parameter_array(container, OutageTimeSeriesParameter(), T)
    multiplier =
        PSI.get_parameter_multiplier_array(container, OutageTimeSeriesParameter(), T)

    for t in time_steps
        for (ix, ic) in enumerate(initial_duration[:, 1])
            name = PSI.get_component_name(ic)

            # Minimum Up-time Constraint
            expr_up = JuMP.GenericAffExpr{Float64, JuMP.VariableRef}(0)
            for i in (t - duration_data[ix].up + 1):t
                if i in time_steps
                    i = Int64(i)
                    JuMP.add_to_expression!(expr_up, varon[name, i])
                end
            end
            if t <= duration_data[ix].up
                expr_up += PSI.get_value(ic)
            end

            con_up[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                varstop[name, t] * duration_data[ix].up <=
                expr_up + (
                    1 * duration_data[ix].up -
                    param[name, t] * duration_data[ix].up
                )
            )

            # Minimum Down-time Constraint
            expr_dn = JuMP.GenericAffExpr{Float64, JuMP.VariableRef}(0)
            for i in (t - duration_data[ix].down + 1):t
                if i in time_steps
                    i = Int64(i)
                    JuMP.add_to_expression!(expr_dn, (1 - varon[name, i]))
                end
            end
            if t <= duration_data[ix].down
                expr_dn += PSI.get_value(initial_duration[ix, 2])
            end

            if t == 1
                expr_dn += (
                    duration_data[ix].down -
                    PSI.get_value(initial_duration[ix, 3]) * duration_data[ix].down
                )
            else
                expr_dn += (
                    duration_data[ix].down -
                    param[name, t - 1] * duration_data[ix].down
                )
            end
            con_down[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                varstart[name, t] * duration_data[ix].down <= expr_dn
            )
        end
    end
    return
end
