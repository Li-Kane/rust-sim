#![allow(dead_code)]

use std::io::Cursor;
use tract_onnx::prelude::*;

/// Raw bytes of the Spot ONNX locomotion policy embedded at compile time for native & WASM.
pub const POLICY_BYTES: &[u8] = include_bytes!("../../assets/spot_policy.onnx");

/// Dimension of the observation vector expected by the policy.
pub const OBSERVATION_DIM: usize = 48;

/// Dimension of the action vector produced by the policy.
pub const ACTION_DIM: usize = 12;

/// URDF to policy joint index remapping.
pub const URDF_TO_POLICY: [usize; 12] = [0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11];

/// Policy to URDF joint index remapping.
pub const POLICY_TO_URDF: [usize; 12] = [0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11];

/// Spot policy inference runner using tract-onnx.
pub struct SpotPolicy {
    model: Arc<TypedRunnableModel>,
}

impl SpotPolicy {
    /// Loads the policy model from an in-memory byte slice.
    pub fn from_bytes(bytes: &[u8]) -> TractResult<Self> {
        let mut reader = Cursor::new(bytes);
        let model = tract_onnx::onnx()
            .model_for_read(&mut reader)?
            .into_optimized()?
            .into_runnable()?;

        Ok(Self { model })
    }

    /// Loads the embedded default `spot_policy.onnx`.
    pub fn load_default() -> TractResult<Self> {
        Self::from_bytes(POLICY_BYTES)
    }

    /// Evaluates the policy forward pass given a 48-float observation vector.
    /// Returns the 12 raw action values (policy joint ordering).
    pub fn step(&self, obs: &[f32; OBSERVATION_DIM]) -> TractResult<[f32; ACTION_DIM]> {
        let tensor = Tensor::from_shape(&[1, OBSERVATION_DIM], obs)?;
        let output = self.model.run(tvec!(tensor.into()))?;
        let plain = output[0].try_as_plain()?;
        let actions_slice = plain.as_slice::<f32>()?;

        let mut actions = [0.0f32; ACTION_DIM];
        actions.copy_from_slice(actions_slice);
        Ok(actions)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_load_and_run_policy() {
        let policy = SpotPolicy::load_default().expect("Failed to load embedded policy");
        let obs = [0.0f32; OBSERVATION_DIM];
        let actions = policy.step(&obs).expect("Policy inference failed");

        assert_eq!(actions.len(), ACTION_DIM);
        // Verify output produces finite numbers
        for val in actions {
            assert!(!val.is_nan());
        }
        println!("Policy output: {:?}", actions);
    }
}
