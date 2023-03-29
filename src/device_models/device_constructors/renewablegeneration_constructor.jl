function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.DeviceModel{R, D},
    ::Type{S},
) where {R <: PSY.RenewableGen, D <: RenewableDispatchOutages, S <: PM.AbstractPowerModel}
    devices = PSI.get_available_components(R, sys)

    PSI.add_variables!(container, PSI.ActivePowerVariable, devices, D())
    PSI.add_variables!(container, PSI.ReactivePowerVariable, devices, D())
    # PSI.add_variables!(container, OutageVariable, devices, D())

    PSI.add_parameters!(container, PSI.ActivePowerTimeSeriesParameter, devices, model)
    PSI.add_parameters!(container, OutageTimeSeriesParameter, devices, model)

    PSI.add_expressions!(container, PSI.ProductionCostExpression, devices, model)

    # Expression
    PSI.add_to_expression!(
        container,
        PSI.ActivePowerBalance,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
    )
    PSI.add_to_expression!(
        container,
        PSI.ReactivePowerBalance,
        PSI.ReactivePowerVariable,
        devices,
        model,
        S,
    )
    if PSI.has_service_model(model)
        PSI.add_to_expression!(
            container,
            PSI.ActivePowerRangeExpressionLB,
            PSI.ActivePowerVariable,
            devices,
            model,
            S,
        )
        PSI.add_to_expression!(
            container,
            PSI.ActivePowerRangeExpressionUB,
            PSI.ActivePowerVariable,
            devices,
            model,
            S,
        )
    end
end

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ModelConstructStage,
    model::PSI.DeviceModel{R, D},
    ::Type{S},
) where {R <: PSY.RenewableGen, D <: RenewableDispatchOutages, S <: PM.AbstractPowerModel}
    devices = PSI.get_available_components(R, sys)

    if PSI.has_service_model(model)
        PSI.add_constraints!(
            container,
            PSI.ActivePowerVariableLimitsConstraint,
            PSI.ActivePowerRangeExpressionUB,
            devices,
            model,
            S,
        )
    else
        PSI.add_constraints!(
            container,
            PSI.ActivePowerVariableLimitsConstraint,
            PSI.ActivePowerVariable,
            devices,
            model,
            S,
        )
    end
    PSI.add_constraints!(
        container,
        PSI.ReactivePowerVariableLimitsConstraint,
        PSI.ReactivePowerVariable,
        devices,
        model,
        S,
    )
    PSI.add_constraints!(container, OutageUpperBoundConstraint, devices, model, S)

    PSI.add_feedforward_constraints!(container, model, devices)

    PSI.objective_function!(container, devices, model, S)

    PSI.add_constraint_dual!(container, sys, model)

    return
end

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.DeviceModel{R, D},
    ::Type{S},
) where {
    R <: PSY.RenewableGen,
    D <: RenewableDispatchOutages,
    S <: PM.AbstractActivePowerModel,
}
    devices = PSI.get_available_components(R, sys)

    PSI.add_variables!(container, PSI.ActivePowerVariable, devices, D())
    # PSI.add_variables!(container, OutageVariable, devices, D())

    PSI.add_parameters!(container, PSI.ActivePowerTimeSeriesParameter, devices, model)
    PSI.add_parameters!(container, OutageTimeSeriesParameter, devices, model)

    PSI.add_expressions!(container, PSI.ProductionCostExpression, devices, model)

    # Expression
    PSI.add_to_expression!(
        container,
        PSI.ActivePowerBalance,
        PSI.ActivePowerVariable,
        devices,
        model,
        S,
    )
    if PSI.has_service_model(model)
        PSI.add_to_expression!(
            container,
            PSI.ActivePowerRangeExpressionLB,
            PSI.ActivePowerVariable,
            devices,
            model,
            S,
        )
        PSI.add_to_expression!(
            container,
            PSI.ActivePowerRangeExpressionUB,
            PSI.ActivePowerVariable,
            devices,
            model,
            S,
        )
    end
end

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ModelConstructStage,
    model::PSI.DeviceModel{R, D},
    ::Type{S},
) where {
    R <: PSY.RenewableGen,
    D <: RenewableDispatchOutages,
    S <: PM.AbstractActivePowerModel,
}
    devices = PSI.get_available_components(R, sys)

    if PSI.has_service_model(model)
        PSI.add_constraints!(
            container,
            PSI.ActivePowerVariableLimitsConstraint,
            PSI.ActivePowerRangeExpressionUB,
            devices,
            model,
            S,
        )
    else
        PSI.add_constraints!(
            container,
            PSI.ActivePowerVariableLimitsConstraint,
            PSI.ActivePowerVariable,
            devices,
            model,
            S,
        )
    end
    PSI.add_constraints!(container, OutageUpperBoundConstraint, devices, model, S)

    PSI.add_feedforward_constraints!(container, model, devices)

    PSI.objective_function!(container, devices, model, S)

    PSI.add_constraint_dual!(container, sys, model)

    return
end
