#!/usr/bin/env python3
"""Epoch authorization, revocation and raw-command bypass regressions."""
import copy
import importlib.util
from pathlib import Path
import unittest
from unittest import mock
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from navigo_epoch_msgs.msg import LocalizationEpoch, NavigationIntent, NavExecutionState, EpochCommand

SPEC = importlib.util.spec_from_file_location('epoch_gate', Path(__file__).resolve().parents[1] / 'scripts/nav_safety_gate.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
NOW = 1_000_000_000_000


def messages(gate='gate', boot='boot'):
    loc = LocalizationEpoch(schema_version=1, heartbeat_sequence=1, lifecycle_enabled=True,
                            health=1, fusion_ready=True, output_ready=True, base_frame_id='base_link')
    loc.identity.process_session_id = 'localizer'
    loc.identity.map_loaded_instance = 'map-instance'
    loc.identity.epoch = loc.identity.commits = 1
    loc.output_pose.header.frame_id = 'map'
    loc.output_pose.header.stamp.sec = 10
    loc.output_pose.pose.orientation.w = 1.
    intent = NavigationIntent(heartbeat_sequence=1, active=True, boot_id=boot, source_steady_time_ns=NOW)
    intent.token.localization = copy.deepcopy(loc.identity)
    intent.token.navigation_session_id = 'navigator'
    intent.token.task_sequence = intent.token.plan_sequence = 1
    intent.token.gate_session_id = gate
    execution = NavExecutionState(token=copy.deepcopy(intent.token), controller_session_id='controller',
        heartbeat_sequence=1, active=True, boot_id=boot, source_steady_time_ns=NOW)
    command = EpochCommand(token=copy.deepcopy(intent.token), controller_session_id='controller',
        command_sequence=1, boot_id=boot, source_steady_time_ns=NOW, max_age_ms=300,
        relay_session_id='smoother', relay_sequence=1)
    command.velocity.linear.x = .1
    return loc, intent, execution, command


def prime(policy, bundle=None):
    bundle = bundle or messages(policy.gate_session, policy.boot_id)
    loc, intent, execution, command = bundle
    policy.localization(loc, NOW)
    policy.navigation_intent(intent, NOW)
    policy.execution_state(execution, NOW)
    return bundle


class PolicyTest(unittest.TestCase):
    def setUp(self):
        self.policy = MODULE.EpochGatePolicy('gate', 'boot')
        self.loc, self.intent, self.execution, self.command = prime(self.policy)

    def test_command_needs_matching_installed_path_not_just_new_epoch(self):
        self.policy.execution = None
        self.assertEqual(self.policy.accept_command(self.command, NOW), 'path_not_installed')
        self.policy.execution_state(self.execution, NOW)
        self.assertIsNone(self.policy.accept_command(self.command, NOW))

    def test_relay_cannot_renew_source_age_and_reordering_is_rejected(self):
        self.assertIsNone(self.policy.accept_command(self.command, NOW))
        self.assertEqual(self.policy.accept_command(self.command, NOW), 'relay_sequence_regression')
        old_source = copy.deepcopy(self.command)
        old_source.relay_sequence = 2
        self.loc.heartbeat_sequence = 2
        self.loc.output_pose.header.stamp.nanosec = 350_000_000
        self.intent.heartbeat_sequence = self.execution.heartbeat_sequence = 2
        self.intent.source_steady_time_ns = self.execution.source_steady_time_ns = NOW + 350_000_000
        self.policy.localization(self.loc, NOW + 350_000_000)
        self.policy.navigation_intent(self.intent, NOW + 350_000_000)
        self.policy.execution_state(self.execution, NOW + 350_000_000)
        self.assertEqual(self.policy.accept_command(old_source, NOW + 350_000_000), 'command_identity_or_age')

    def test_heartbeat_cannot_renew_frozen_output_or_regress_it(self):
        current = copy.deepcopy(self.loc)
        current.heartbeat_sequence = 2
        self.policy.localization(current, NOW + 410_000_000)
        self.assertEqual(self.policy.authority_reason(NOW + 410_000_000), 'localization_not_ready')
        current.heartbeat_sequence = 3
        current.output_pose.header.stamp.sec = 9
        self.policy.localization(current, NOW + 420_000_000)
        current.heartbeat_sequence = 4
        current.output_pose.header.stamp.sec = 11
        self.policy.localization(current, NOW + 430_000_000)
        self.assertTrue(self.policy.output_regressed)

    def test_notready_zero_stamp_and_reload_revocation_preserve_watermarks(self):
        revoked = copy.deepcopy(self.loc)
        revoked.heartbeat_sequence = 2
        revoked.health, revoked.output_ready = 3, False
        revoked.identity.map_loaded_instance = ''
        revoked.identity.epoch = revoked.identity.commits = 0
        revoked.output_pose.header.stamp.sec = 0
        self.policy.localization(revoked, NOW + 100_000_000)
        self.assertFalse(self.policy.loc.output_ready)
        self.assertFalse(self.policy.output_regressed)
        restored = copy.deepcopy(self.loc)
        restored.heartbeat_sequence = 3
        self.policy.localization(restored, NOW + 410_000_000)
        self.assertEqual(self.policy.output_advanced, NOW)
        self.assertEqual(self.policy.authority_reason(NOW + 410_000_000), 'localization_not_ready')
        lower = copy.deepcopy(restored)
        lower.heartbeat_sequence = 4
        lower.identity.epoch = 0
        self.policy.localization(lower, NOW + 420_000_000)
        self.assertEqual(self.policy.loc.identity.epoch, 1)

    def test_source_and_interpolation_sequences_are_not_interchangeable(self):
        self.policy.accept_command(self.command, NOW)
        changed = copy.deepcopy(self.command)
        changed.relay_sequence = 2
        changed.source_steady_time_ns += 1
        self.assertEqual(self.policy.accept_command(changed, NOW + 1), 'source_sequence_regression')
        changed.command_sequence = 2
        self.assertIsNone(self.policy.accept_command(changed, NOW + 1))

    def test_smoother_restart_retires_old_relay_even_with_a_fresh_source(self):
        self.policy.accept_command(self.command, NOW)
        restarted = copy.deepcopy(self.command)
        restarted.relay_session_id = 'new-smoother'
        self.assertIsNone(self.policy.accept_command(restarted, NOW))
        old = copy.deepcopy(self.command); old.relay_sequence += 1
        self.assertEqual(self.policy.accept_command(old, NOW), 'retired_relay_session')

    def test_old_identity_gate_and_boot_are_rejected(self):
        for change in ('epoch', 'gate', 'boot'):
            command = copy.deepcopy(self.command)
            if change == 'epoch': command.token.localization.epoch += 1
            if change == 'gate': command.token.gate_session_id = 'old-gate'
            if change == 'boot': command.boot_id = 'other-host'
            self.assertIsNotNone(self.policy.accept_command(command, NOW))

    def test_map_change_and_retired_localizer_cannot_restore_old_grid(self):
        newer = copy.deepcopy(self.loc)
        newer.identity.process_session_id = 'new-localizer'
        self.policy.localization(newer, NOW + 1)
        replay = copy.deepcopy(self.loc)
        replay.heartbeat_sequence = 100
        self.policy.localization(replay, NOW + 2)
        self.assertEqual(self.policy.loc.identity.process_session_id, 'new-localizer')
        newer = copy.deepcopy(newer)
        newer.heartbeat_sequence = 2
        newer.identity.map_loaded_instance = 'other-map'
        self.policy.localization(newer, NOW + 3)
        self.assertTrue(self.policy.map_changed)

    def test_plan_handoff_wait_does_not_demand_another_challenge(self):
        self.policy.accept_command(self.command, NOW)
        intent = copy.deepcopy(self.intent)
        intent.heartbeat_sequence += 1
        intent.token.plan_sequence += 1
        self.policy.navigation_intent(intent, NOW)
        self.assertEqual(self.policy.authority_reason(NOW), 'path_not_installed')
        self.assertIsNone(self.policy.revocation_reason(NOW))

    def test_duplicate_intent_does_not_extend_lease(self):
        self.policy.navigation_intent(copy.deepcopy(self.intent), NOW + 399_000_000)
        self.assertEqual(self.policy.intent_receipt, NOW)


class NodeRevocationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls): rclpy.init(args=[])
    @classmethod
    def tearDownClass(cls): rclpy.shutdown()

    def setUp(self):
        self.node = MODULE.NavSafetyGate(parameter_overrides=[Parameter('enable_epoch_contract', value=True)])
        self.outputs = []
        self.node.publish_cmd = lambda command, status: self.outputs.append((command, status))
        self.bundle = prime(self.node.epoch_policy)
        self.clock = mock.patch.object(MODULE.time, 'monotonic_ns', return_value=NOW)
        self.clock.start()
        self.node.epoch_command_callback(self.bundle[-1])

    def tearDown(self):
        self.clock.stop()
        self.node.destroy_node()

    def test_loss_rotates_once_and_same_epoch_old_plan_stays_blocked(self):
        previous = self.node.epoch_policy.gate_session
        lost = copy.deepcopy(self.bundle[0])
        lost.heartbeat_sequence = 2
        lost.health, lost.output_ready = 3, False
        self.node.epoch_authority_callback('localization', lost)
        renewed = self.node.epoch_policy.gate_session
        self.assertNotEqual(previous, renewed)
        self.node.enforce_epoch()
        self.assertEqual(self.node.epoch_policy.gate_session, renewed)
        restored = copy.deepcopy(self.bundle[0]); restored.heartbeat_sequence = 3
        self.node.epoch_authority_callback('localization', restored)
        command = copy.deepcopy(self.bundle[-1]); command.command_sequence = command.relay_sequence = 2
        command.source_steady_time_ns += 1
        self.node.epoch_command_callback(command)
        self.assertEqual(self.outputs[-1][0], Twist())

    def test_estop_requires_new_plan_and_raw_twist_is_never_wrapped(self):
        previous = self.node.epoch_policy.gate_session
        self.node.emergency_stop_callback(Bool(data=True))
        self.assertNotEqual(previous, self.node.epoch_policy.gate_session)
        self.node.emergency_stop_callback(Bool(data=False))
        raw = Twist(); raw.linear.x = .1
        self.node.cmd_vel_callback(raw)
        self.assertEqual(self.outputs[-1][0], Twist())
        self.assertIsNone(self.node.cmd_vel_sub)

    def test_normal_terminal_seals_token_without_challenge_or_ttl_rotation(self):
        previous = self.node.epoch_policy.gate_session
        terminal = copy.deepcopy(self.bundle[2])
        terminal.heartbeat_sequence += 1
        terminal.active, terminal.reason = False, 'goal_reached'
        self.node.epoch_authority_callback('execution_state', terminal)
        self.assertFalse(self.node.epoch_armed)
        self.assertEqual(self.outputs[-1][0], Twist())
        self.assertIsNone(self.node.epoch_policy.command)
        for active, reason in ((False, 'controller_idle'), (True, 'installed')):
            terminal.heartbeat_sequence += 1
            terminal.active, terminal.reason = active, reason
            self.node.epoch_authority_callback('execution_state', terminal)
            replay = copy.deepcopy(self.bundle[-1])
            replay.command_sequence += 1
            replay.relay_sequence += 1
            self.node.epoch_command_callback(replay)
            self.assertEqual(self.outputs[-1][0], Twist())
        self.node.enforce_epoch()
        self.assertEqual(previous, self.node.epoch_policy.gate_session)
        self.assertFalse(self.node.epoch_armed)

    def test_progress_failure_seals_old_token_and_allows_fresh_same_task_backup(self):
        previous=self.node.epoch_policy.gate_session
        loc,intent,execution,command=copy.deepcopy(self.bundle)
        execution.heartbeat_sequence+=1;execution.active=False;execution.reason='progress_failed'
        self.node.epoch_authority_callback('execution_state',execution)
        self.assertEqual(self.outputs[-1][0],Twist())
        self.assertEqual(previous,self.node.epoch_policy.gate_session)
        # The BT consumes FAILURE and revokes intent while clearing/waiting.
        intent.heartbeat_sequence+=1;intent.active=False
        self.node.epoch_authority_callback('navigation_intent',intent)
        self.node.enforce_epoch()
        self.assertEqual(previous,self.node.epoch_policy.gate_session)
        # Neither a reactivated execution nor an old raw/relay command revives it.
        execution.heartbeat_sequence+=1;execution.active=True;execution.reason='installed'
        self.node.epoch_authority_callback('execution_state',execution)
        intent.heartbeat_sequence+=1;intent.active=True
        self.node.epoch_authority_callback('navigation_intent',intent)
        command.command_sequence+=1;command.relay_sequence+=1
        self.node.epoch_command_callback(command)
        self.assertEqual(self.outputs[-1][0],Twist())
        raw=Twist();raw.linear.x=-.08;self.node.cmd_vel_callback(raw)
        self.assertEqual(self.outputs[-1][0],Twist())
        # A new BACKUP request still needs full normal admission and installation.
        intent.heartbeat_sequence+=1;intent.token.plan_sequence+=1
        intent.token.execution_kind=1;intent.token.backup_max_speed=.08
        intent.token.execution_deadline_ns=NOW+5_000_000_000
        self.node.epoch_authority_callback('navigation_intent',intent)
        command.token=copy.deepcopy(intent.token);command.command_sequence+=1;command.relay_sequence+=1
        command.source_steady_time_ns+=1
        command.velocity.linear.x=-.08;command.source_motion_phase=command.PHASE_TRANSLATE
        self.node.epoch_command_callback(command)
        self.assertEqual(self.outputs[-1][0],Twist())
        execution.token=copy.deepcopy(intent.token);execution.heartbeat_sequence+=1
        self.node.epoch_authority_callback('execution_state',execution)
        with mock.patch.object(MODULE.time,'monotonic_ns',return_value=NOW+1):
            self.node.epoch_command_callback(command)
        self.assertEqual(self.outputs[-1][0].linear.x,-.08)
        self.assertEqual(self.node.epoch_policy.intent.token.task_sequence,1)
        self.assertEqual(previous,self.node.epoch_policy.gate_session)

    def test_sealed_progress_failure_does_not_mask_localization_loss_or_timeout(self):
        for failure in ('lost', 'timeout', 'estop'):
            with self.subTest(failure=failure):
                policy=MODULE.EpochGatePolicy('gate','boot');bundle=prime(policy)
                self.node.epoch_policy=policy;self.node.epoch_armed=True
                ended=copy.deepcopy(bundle[2]);ended.heartbeat_sequence+=1
                ended.active=False;ended.reason='progress_failed'
                self.node.epoch_authority_callback('execution_state',ended)
                self.assertEqual(policy.gate_session,'gate')
                if failure=='lost':
                    lost=copy.deepcopy(bundle[0]);lost.heartbeat_sequence+=1;lost.health=3
                    self.node.epoch_authority_callback('localization',lost)
                elif failure=='timeout':
                    with mock.patch.object(MODULE.time,'monotonic_ns',return_value=NOW+500_000_000):
                        self.node.enforce_epoch()
                else:
                    self.node.emergency_stop_callback(Bool(data=True))
                    self.node.emergency_stop_callback(Bool(data=False))
                self.assertNotEqual(policy.gate_session,'gate')
                new=copy.deepcopy(bundle[1]);new.heartbeat_sequence+=1;new.token.plan_sequence+=1
                new.token.execution_kind=1;new.token.backup_max_speed=.08
                new.token.execution_deadline_ns=NOW+5_000_000_000
                self.node.epoch_authority_callback('navigation_intent',new)
                self.assertNotEqual(policy.intent.token,new.token)
                self.assertEqual(self.outputs[-1][0],Twist())

    def test_terminal_requires_same_identity_challenge_and_fresh_inactive_intent(self):
        for failure in ('epoch','gate','intent_timeout'):
            policy=MODULE.EpochGatePolicy('gate','boot');loc,intent,execution,_=prime(policy)
            execution=copy.deepcopy(execution);execution.heartbeat_sequence+=1
            execution.active=False;execution.reason='progress_failed';policy.execution_state(execution,NOW)
            if failure=='epoch':
                loc=copy.deepcopy(loc);loc.heartbeat_sequence+=1;loc.identity.epoch+=1
                policy.localization(loc,NOW)
            elif failure=='gate':policy.gate_session='new-gate'
            else:
                loc=copy.deepcopy(loc);loc.heartbeat_sequence+=1;loc.output_pose.header.stamp.sec+=1
                policy.localization(loc,NOW+500_000_000)
            now=NOW+500_000_000 if failure=='intent_timeout' else NOW
            self.assertEqual(policy.authority_reason(now),'intent_not_ready')
            self.assertEqual(policy.revocation_reason(now),'intent_not_ready')

    def test_nonprogress_revocations_do_not_receive_terminal_exception(self):
        for reason in ('Epoch authority or source TF revoked','Execution odometry unavailable/stale/invalid','Costmap observations stale'):
            with self.subTest(reason=reason):
                policy=MODULE.EpochGatePolicy('gate','boot');bundle=prime(policy)
                ended=copy.deepcopy(bundle[2]);ended.heartbeat_sequence+=1
                ended.active=False;ended.reason=reason;policy.execution_state(ended,NOW)
                self.assertFalse(policy.terminal_current())
                self.assertEqual(policy.revocation_reason(NOW),'path_not_installed')
        expired=copy.deepcopy(self.bundle[2]);expired.active=False;expired.reason='progress_failed'
        expired.heartbeat_sequence+=1;expired.source_steady_time_ns=NOW-400_000_000
        self.node.epoch_authority_callback('execution_state',expired)
        self.assertFalse(self.node.epoch_policy.terminal_current())

    def test_new_task_can_install_after_terminal(self):
        terminal = copy.deepcopy(self.bundle[2])
        terminal.heartbeat_sequence += 1
        terminal.active, terminal.reason = False, 'goal_reached'
        self.node.epoch_authority_callback('execution_state', terminal)
        intent = copy.deepcopy(self.bundle[1])
        intent.heartbeat_sequence += 1
        intent.token.task_sequence += 1
        self.node.epoch_authority_callback('navigation_intent', intent)
        execution = copy.deepcopy(self.bundle[2])
        execution.token = copy.deepcopy(intent.token)
        execution.heartbeat_sequence = 3
        self.node.epoch_authority_callback('execution_state', execution)
        command = copy.deepcopy(self.bundle[-1])
        command.token = copy.deepcopy(intent.token)
        command.command_sequence = command.relay_sequence = 2
        command.source_steady_time_ns += 1
        with mock.patch.object(MODULE.time, 'monotonic_ns', return_value=NOW + 1):
            self.node.epoch_command_callback(command)
        self.assertEqual(self.outputs[-1][0].linear.x, .1)
        self.assertTrue(self.node.epoch_armed)

    def test_uninstalled_terminal_cannot_disarm_active_execution(self):
        terminal = copy.deepcopy(self.bundle[2])
        terminal.controller_session_id = 'uninstalled-controller'
        terminal.heartbeat_sequence += 1
        terminal.active, terminal.reason = False, 'goal_reached'
        self.node.epoch_authority_callback('execution_state', terminal)
        self.assertFalse(self.node.epoch_policy.terminal_current())
        self.assertNotEqual(self.node.epoch_policy.gate_session, self.bundle[1].token.gate_session_id)

    def test_initial_wait_does_not_rotate_challenge(self):
        self.node.epoch_armed = False
        self.node.epoch_policy.loc = None
        previous = self.node.epoch_policy.gate_session
        for _ in range(3): self.node.enforce_epoch()
        self.assertEqual(previous, self.node.epoch_policy.gate_session)



class BackupPolicyTest(unittest.TestCase):
    def bundle(self):
        policy = MODULE.EpochGatePolicy('gate','boot')
        loc,intent,execution,command = messages()
        intent.token.execution_kind = 1
        intent.token.backup_max_speed = .1
        intent.token.execution_deadline_ns = NOW + 200_000_000
        execution.token = copy.deepcopy(intent.token)
        command.token = copy.deepcopy(intent.token)
        command.velocity.linear.x = -.08
        command.source_motion_phase = 2
        prime(policy,(loc,intent,execution,command))
        return policy,loc,intent,execution,command

    def test_slow_watchdog_does_not_admit_backup(self):
        policy,loc,intent,execution,command=self.bundle()
        slow=MODULE.EpochGatePolicy('gate','boot');slow.backup_watchdog_period_ns=100_000_000
        slow.localization(loc,NOW);slow.navigation_intent(intent,NOW)
        self.assertIsNone(slow.intent)
        normal=messages();slow.navigation_intent(normal[1],NOW)
        self.assertIsNotNone(slow.intent)

    def test_reverse_requires_explicit_backup(self):
        policy = MODULE.EpochGatePolicy('gate','boot')
        _,_,_,command = prime(policy)
        command.velocity.linear.x = -.01
        self.assertEqual(policy.accept_command(command,NOW),'tracking_reverse_forbidden')
        policy,_,_,_,command = self.bundle()
        self.assertIsNone(policy.accept_command(command,NOW))

    def test_backup_limits_and_phase(self):
        for axis,value in [('x',-.101),('y',.01)]:
            policy,_,_,_,command = self.bundle()
            setattr(command.velocity.linear,axis,value)
            self.assertEqual(policy.accept_command(command,NOW),'backup_velocity_bound')
        policy,_,_,_,command = self.bundle()
        command.velocity.angular.z=.01
        self.assertEqual(policy.accept_command(command,NOW),'mixed_motion_axes')

    def test_deadline_cannot_be_renewed_with_same_sequence(self):
        policy,_,intent,_,command = self.bundle()
        modified = copy.deepcopy(intent)
        modified.heartbeat_sequence += 1
        modified.token.execution_deadline_ns += 100_000_000
        policy.navigation_intent(modified,NOW+1)
        self.assertEqual(policy.intent.token.execution_deadline_ns,intent.token.execution_deadline_ns)
        self.assertEqual(policy.accept_command(command,NOW+200_000_000),'backup_deadline')

    def test_completion_seals_backup_token(self):
        policy,_,_,execution,command = self.bundle()
        execution.active=False;execution.reason='backup_finished';execution.heartbeat_sequence+=1
        policy.execution_state(execution,NOW+1)
        self.assertTrue(policy.terminal_current())
        self.assertEqual(policy.accept_command(command,NOW+2),'task_completed')

    def test_localization_loss_cancels_backup(self):
        policy,loc,_,_,command = self.bundle()
        loc.heartbeat_sequence+=1;loc.health=3;loc.output_ready=False
        policy.localization(loc,NOW+1)
        self.assertEqual(policy.accept_command(command,NOW+2),'localization_not_ready')

class MotionPhaseGateTest(unittest.TestCase):
    def test_curve_tracking_can_brake_before_rotation(self):
        policy=MODULE.EpochGatePolicy('gate','boot')
        _,_,_,old=prime(policy)
        old.source_motion_phase=4;old.velocity.angular.z=.1
        self.assertIsNone(policy.accept_command(old,NOW))
        braking=copy.deepcopy(old)
        braking.command_sequence=2;braking.source_steady_time_ns=NOW+1;braking.relay_sequence=2
        braking.source_motion_phase=3;braking.transition_braking=True;braking.braking_from_phase=4
        braking.velocity.linear.x=.05;braking.velocity.angular.z=.05
        self.assertIsNone(policy.accept_command(braking,NOW+1))

    def test_braking_cannot_be_interrupted_before_zero(self):
        policy=MODULE.EpochGatePolicy('gate','boot')
        _,_,_,old=prime(policy);old.source_motion_phase=3;old.velocity.linear.x=0.;old.velocity.angular.z=.1
        self.assertIsNone(policy.accept_command(old,NOW))
        braking=copy.deepcopy(old);braking.relay_sequence=2;braking.source_motion_phase=1
        braking.transition_braking=True;braking.braking_from_phase=3;braking.velocity.angular.z=.05
        self.assertIsNone(policy.accept_command(braking,NOW+1))
        resume=copy.deepcopy(old);resume.relay_sequence=3
        self.assertEqual(policy.accept_command(resume,NOW+2),'braking_zero_handoff_required')
        braking.relay_sequence=3;braking.velocity.angular.z=0.
        self.assertIsNone(policy.accept_command(braking,NOW+3))
        resume.relay_sequence=4
        self.assertIsNone(policy.accept_command(resume,NOW+4))

    def test_braking_rejects_forged_source_amplification_and_sign(self):
        for field in ('source','amplitude','sign'):
            policy=MODULE.EpochGatePolicy('gate','boot')
            _,_,_,old=prime(policy);old.source_motion_phase=3;old.velocity.linear.x=0.;old.velocity.angular.z=.1
            self.assertIsNone(policy.accept_command(old,NOW))
            braking=copy.deepcopy(old);braking.relay_sequence=2
            braking.source_motion_phase=1;braking.transition_braking=True;braking.braking_from_phase=3
            braking.velocity.angular.z=.05
            if field=='source':braking.braking_from_phase=4
            if field=='amplitude':braking.velocity.angular.z=.11
            if field=='sign':braking.velocity.angular.z=-.01
            expected='braking_source_mismatch' if field=='source' else 'braking_amplitude_or_sign'
            self.assertEqual(policy.accept_command(braking,NOW+1),expected)

if __name__ == '__main__': unittest.main()
