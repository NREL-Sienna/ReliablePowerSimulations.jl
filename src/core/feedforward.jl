struct SemiContinuousOutageFeedforward <: PSI.AbstractAffectFeedforward
    optimization_container_key::PSI.OptimizationContainerKey
    affected_values::Vector{<:PSI.OptimizationContainerKey}
    function SemiContinuousOutageFeedforward(;
        component_type::Type{<:PSY.Component},
        source::Type{T},
        affected_values::Vector{DataType},
        meta = PSI.CONTAINER_KEY_EMPTY_META,
    ) where {T}
        values_vector = Vector{PSI.VariableKey}(undef, length(affected_values))
        for (ix, v) in enumerate(affected_values)
            if v <: PSI.VariableType
                values_vector[ix] =
                    PSI.get_optimization_container_key(v(), component_type, meta)
            else
                error(
                    "SemiContinuousOutageFeedforward is only compatible with VariableType affected values",
                )
            end
        end
        new(PSI.get_optimization_container_key(T(), component_type, meta), values_vector)
    end
end

PSI.get_default_parameter_type(::SemiContinuousOutageFeedforward, _) =
    PSI.OnStatusParameter()
PSI.get_optimization_container_key(f::SemiContinuousOutageFeedforward) =
    f.optimization_container_key

function has_semicontinuous_outage_feedforward(
    model::PSI.DeviceModel,
    ::Type{T},
)::Bool where {T <: Union{PSI.VariableType, PSI.ExpressionType}}
    if isempty(model.feedforwards)
        return false
    end
    sc_feedforwards =
        [x for x in model.feedforwards if isa(x, SemiContinuousOutageFeedforward)]
    if isempty(sc_feedforwards)
        return false
    end

    keys = PSI.get_affected_values(sc_feedforwards[1])

    return T ∈ PSI.get_entry_type.(keys)
end

function has_semicontinuous_outage_feedforward(
    model::PSI.DeviceModel,
    ::Type{T},
)::Bool where {
    T <: Union{PSI.ActivePowerRangeExpressionUB, PSI.ActivePowerRangeExpressionLB},
}
    return has_semicontinuous_outage_feedforward(model, PSI.ActivePowerVariable)
end

function PSI.add_feedforward_arguments!(
    container::PSI.OptimizationContainer,
    model::PSI.DeviceModel,
    devices::IS.FlattenIteratorWrapper{T},
    ff::SemiContinuousOutageFeedforward,
) where {T <: PSY.Component}
    parameter_type = PSI.get_default_parameter_type(ff, T)
    source_key = PSI.get_optimization_container_key(ff)
    PSI.add_parameters!(container, parameter_type, source_key, model, devices)
    PSI.add_variables!(
        container,
        AuxiliaryOnVariable,
        devices,
        PSI.get_formulation(model)(),
    )
    PSI.add_variables!(
        container,
        AuxiliaryStartVariable,
        devices,
        PSI.get_formulation(model)(),
    )
    PSI.add_variables!(
        container,
        AuxiliaryStopVariable,
        devices,
        PSI.get_formulation(model)(),
    )
    return
end

function PSI.add_feedforward_constraints!(
    container::PSI.OptimizationContainer,
    model::PSI.DeviceModel,
    devices::IS.FlattenIteratorWrapper{T},
    ff::SemiContinuousOutageFeedforward,
) where {T <: PSY.Component}
    parameter_type = PSI.get_default_parameter_type(ff, T)
    time_steps = PSI.get_time_steps(container)
    for var in PSI.get_affected_values(ff)
        variable = PSI.get_variable(container, var)
        axes = JuMP.axes(variable)
        IS.@assert_op axes[1] == [PSY.get_name(d) for d in devices]
        IS.@assert_op axes[2] == time_steps
        # If the variable was a lower bound != 0, not removing the LB can cause infeasibilities
        for v in variable
            if JuMP.has_lower_bound(v) && JuMP.lower_bound(v) > 0.0
                @debug "lb reset" JuMP.lower_bound(v) v _group =
                    PSI.LOG_GROUP_FEEDFORWARDS_CONSTRUCTION
                JuMP.set_lower_bound(v, 0.0)
            end
        end
        add_sc_outage_feedforward_constraints!(
            container,
            FeedforwardSemiContinousOutageConstraint,
            var,
            devices,
            model,
        )
        add_device_status_feedforward_constraints!(
            container,
            FeedforwardDeviceStatusConstraint,
            var,
            devices,
            model,
        )
        add_aux_commitment_feedforward_constraints!(
            container,
            FeedforwardAuxCommitmentConstraint,
            var,
            devices,
            model,
        )
    end
    return
end

function add_sc_outage_feedforward_constraints!(
    container::PSI.OptimizationContainer,
    ::Type{T},
    ::PSI.VariableKey{U, V},
    devices::IS.FlattenIteratorWrapper{V},
    model::PSI.DeviceModel{V, W},
) where {
    T <: FeedforwardSemiContinousOutageConstraint,
    U <: Union{PSI.ActivePowerVariable, PSI.PowerAboveMinimumVariable},
    V <: PSY.Component,
    W <: PSI.AbstractDeviceFormulation,
}
    time_steps = PSI.get_time_steps(container)
    names = [PSY.get_name(d) for d in devices]
    constraint_lb = PSI.add_constraints_container!(
        container,
        T(),
        V,
        names,
        time_steps,
        meta = "lb",
    )
    constraint_ub = PSI.add_constraints_container!(
        container,
        T(),
        V,
        names,
        time_steps,
        meta = "ub",
    )
    array_lb = PSI.get_expression(container, PSI.ActivePowerRangeExpressionLB(), V)
    array_ub = PSI.get_expression(container, PSI.ActivePowerRangeExpressionUB(), V)
    varon = PSI.get_variable(container, AuxiliaryOnVariable(), V)

    for d in devices
        name = PSY.get_name(d)
        for t in time_steps
            constraint_ub[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                array_ub[name, t] <=
                PSI.get_variable_upper_bound(U(), d, W()) * varon[name, t]
            )
            constraint_lb[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                array_lb[name, t] >=
                PSI.get_variable_lower_bound(U(), d, W()) * varon[name, t]
            )
        end
    end

    return
end

function add_device_status_feedforward_constraints!(
    container::PSI.OptimizationContainer,
    ::Type{T},
    ::PSI.VariableKey{U, V},
    devices::IS.FlattenIteratorWrapper{V},
    model::PSI.DeviceModel{V, W},
) where {
    T <: FeedforwardDeviceStatusConstraint,
    U <: Union{PSI.ActivePowerVariable, PSI.PowerAboveMinimumVariable},
    V <: PSY.Component,
    W <: PSI.AbstractDeviceFormulation,
}
    time_steps = PSI.get_time_steps(container)
    names = [PSY.get_name(d) for d in devices]
    commitment_param = PSI.get_parameter_array(container, PSI.OnStatusParameter(), V)
    outage_parameter = PSI.get_parameter_array(container, OutageTimeSeriesParameter(), V)

    commitment_multiplier =
        PSI.get_parameter_multiplier_array(container, PSI.OnStatusParameter(), V)
    outage_multiplier =
        PSI.get_parameter_multiplier_array(container, OutageTimeSeriesParameter(), V)

    varon = PSI.get_variable(container, AuxiliaryOnVariable(), V)
    cons_commit = PSI.add_constraints_container!(
        container,
        T(),
        V,
        names,
        time_steps,
        meta = "commit",
    )
    cons_outage = PSI.add_constraints_container!(
        container,
        T(),
        V,
        names,
        time_steps,
        meta = "outage",
    )
    cons_aux = PSI.add_constraints_container!(
        container,
        T(),
        V,
        names,
        time_steps,
    )

    for d in devices
        name = PSY.get_name(d)
        for t in time_steps
            commitment_multiplier[name, t] = 1.0
            outage_multiplier[name, t] = 1.0
            cons_commit[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                varon[name, t] <= commitment_param[name, t]
            )
            cons_outage[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                varon[name, t] <= outage_parameter[name, t]
            )
            cons_aux[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                varon[name, t] >= outage_parameter[name, t] + commitment_param[name, t]- 1.0
            )
        end
    end

    return
end

function add_device_status_feedforward_constraints!(
    container::PSI.OptimizationContainer,
    ::Type{T},
    ::PSI.VariableKey{U, V},
    devices::IS.FlattenIteratorWrapper{V},
    model::PSI.DeviceModel{V, W},
) where {
    T <: FeedforwardDeviceStatusConstraint,
    U <: Union{PSI.ActivePowerVariable, PSI.PowerAboveMinimumVariable},
    V <: PSY.Component,
    W <: PSI.AbstractDeviceFormulation,
}
    time_steps = PSI.get_time_steps(container)
    names = [PSY.get_name(d) for d in devices]
    varon = PSI.get_variable(container, AuxiliaryOnVariable(), V)
    varstop = PSI.get_variable(container, AuxiliaryStopVariable(), V)
    varstart = PSI.get_variable(container, AuxiliaryStartVariable(), V)

    cons_start = PSI.add_constraints_container!(
        container,
        T(),
        V,
        names,
        time_steps[2:end],
        meta = "start",
    )
    cons_stop = PSI.add_constraints_container!(
        container,
        T(),
        V,
        names,
        time_steps[2:end],
        meta = "stop",
    )
    cons_bin = PSI.add_constraints_container!(
        container,
        T(),
        V,
        names,
        time_steps,
        meta = "bin",
    )
    cons = PSI.add_constraints_container!(
        container,
        T(),
        V,
        names,
        time_steps,
    )

    for d in devices
        name = PSY.get_name(d)
        for t in time_steps
            cons_bin[name, t] = JuMP.@constraint(
                container.JuMPmodel,
                varstart[name, t] + varstop[name, t] <= 1.0
            )
            if t >= 2
                cons_start[name, t] = JuMP.@constraint(
                    container.JuMPmodel,
                    varon[name, t] - varon[name, t-1] <= varstart[name, t]
                )
                cons_stop[name, t] = JuMP.@constraint(
                    container.JuMPmodel,
                    varon[name, t-1] - varon[name, t] <= varstop[name, t]
                )
                cons[name, t] = JuMP.@constraint(
                    container.JuMPmodel,
                    varon[name, t] - varon[name, t-1] == varstart[name, t] - varstop[name, t]   
                )
            end
        end
    end

    return
end
