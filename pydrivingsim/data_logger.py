import csv
import os
from datetime import datetime


class DataLogger:
    def __init__(self, output_dir="logs", filename=None, sample_dt=0.01, separate_episodes=True):
        self.sample_dt = sample_dt
        self._last_time = None
        self._file = None
        self._writer = None
        self._base_name = None
        self._output_dir = output_dir
        self._separate_episodes = separate_episodes
        self._episode_id = 0
        self._episode_step = 0
        self._episode_start_time = None

        os.makedirs(output_dir, exist_ok=True)
        if filename is None:
            filename = datetime.now().strftime("sim_%Y%m%d_%H%M%S.csv")
        self._base_name = filename
        self.start_episode(0)

    def _should_log(self, time_s):
        if self.sample_dt is None:
            return True
        if self._last_time is None or (time_s - self._last_time) >= self.sample_dt:
            self._last_time = time_s
            return True
        return False

    def _make_path(self, episode_id):
        if not self._separate_episodes:
            return os.path.join(self._output_dir, self._base_name)
        base, ext = os.path.splitext(self._base_name)
        return os.path.join(self._output_dir, f"{base}_ep{episode_id}{ext}")

    def _write_header(self):
        header = [
            "episode_id",
            "episode_time",
            "episode_step",
            "time",
            "action_pedal",
            "action_steer",
            "state_x",
            "state_y",
            "state_psi",
            "state_u",
            "state_v",
            "state_yaw_rate",
            "state_Fz_rr",
            "state_Fz_rl",
            "state_Fz_fr",
            "state_Fz_fl",
            "state_delta",
            "state_omega_rr",
            "state_omega_rl",
            "state_omega_fr",
            "state_omega_fl",
            "state_alpha_rr",
            "state_alpha_rl",
            "state_alpha_fr",
            "state_alpha_fl",
            "state_kappa_rr",
            "state_kappa_rl",
            "state_kappa_fr",
            "state_kappa_fl",
            "state_pedal",
            "dX_x",
            "dX_y",
            "dX_psi",
            "dX_u",
            "dX_v",
            "dX_yaw_rate",
            "dX_Fz_rr",
            "dX_Fz_rl",
            "dX_Fz_fr",
            "dX_Fz_fl",
            "dX_delta",
            "dX_omega_rr",
            "dX_omega_rl",
            "dX_omega_fr",
            "dX_omega_fl",
            "dX_alpha_rr",
            "dX_alpha_rl",
            "dX_alpha_fr",
            "dX_alpha_fl",
            "dX_kappa_rr",
            "dX_kappa_rl",
            "dX_kappa_fr",
            "dX_kappa_fl",
            "dX_pedal",
        ]
        self._writer.writerow(header)
        self._file.flush()

    def start_episode(self, episode_id, start_time_s=None):
        if self._file is not None and self._separate_episodes:
            self._file.flush()
            self._file.close()
        self._episode_id = episode_id
        self._episode_step = 0
        self._episode_start_time = start_time_s
        self.path = self._make_path(episode_id)
        self._file = open(self.path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._write_header()

    def log(self, time_s, state, dX, action, episode_id=None, episode_time=None):
        if not self._should_log(time_s):
            return
        if episode_id is None:
            episode_id = self._episode_id
        if episode_time is None:
            if self._episode_start_time is None:
                episode_time = 0.0
            else:
                episode_time = time_s - self._episode_start_time
        row = [episode_id, episode_time, self._episode_step, time_s, action[0], action[1]]
        row += list(state)
        row += list(dX)
        self._writer.writerow(row)
        self._episode_step += 1

    def close(self):
        if self._file is not None:
            self._file.flush()
            self._file.close()
            self._file = None
            self._writer = None
