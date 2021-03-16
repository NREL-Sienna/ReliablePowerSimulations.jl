struct DeviceDurationConstraintInfo <: PSI.AbstractStartConstraintInfo
    component_name::String
    duration_data::PSI.UpDown
    initial_duration::Tuple{PSI.InitialCondition}
    multiplier::Union{Nothing, Float64}
    timeseries::Union{Nothing, Vector{Float64}}
end
