struct RenewableOutageDispatch <: PSI.AbstractRenewableDispatchFormulation end

function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    T::Type{OutageUpperBoundConstraint},
    U::Type{<:Union{PSI.VariableType, PSI.ExpressionType}},
    devices::IS.FlattenIteratorWrapper{V},
    model::PSI.DeviceModel{V, W},
    X::Type{<:PM.AbstractActivePowerModel},
) where {V <: PSY.RenewableGen, W <: RenewableOutageDispatch}
    device_outage_ub_parameter!(
        container,
        T,
        U,
        devices, 
        model,
        X,
    )
    return
end
