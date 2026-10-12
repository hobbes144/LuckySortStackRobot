using System.Collections.Generic;
using Hazel;
using Hazel.Data;
using LuckySortStack.Planning;

namespace LuckySortStack.Episodes
{
	// Thin wrapper over Lucky's Observer (Hazel.Data.Observer), which records robot state, actions
	// and camera frames to its own LeRobot-style dataset (<project>/DataSessions/session_<time>/).
	// This class only drives episode boundaries and labels: one Observer task per distinct pick
	// ("Pick Large Red -> RED[0]"), so the recording is segmented per pick. Engine-dependent, so it
	// lives in Episodes/ rather than Recording/.
	//
	// Found on the first recorded run (LUCKY_API_NOTES §10):
	//  - StartRecording() opens the first episode (the RobotSandbox sample never calls BeginEpisode()).
	//  - Tasks registered after StartRecording() never reached tasks.parquet (only "Default Task"), so
	//    every possible pick label is registered up front, before recording starts.
	//  - The scene's ObserverConfig.MaxEpisodeDurationSeconds cuts long episodes; it must exceed a full run.
	public sealed class ObserverSession
	{
		private readonly Dictionary<string, int> m_TaskIndex = new Dictionary<string, int>();
		private bool m_EpisodeOpen;
		private int m_Episodes;

		// Observer recording goes through Lucky Hub and is refused when signed out
		// ("HUB: Recording start refused: user is not authenticated"). Check first and say so plainly;
		// signing in is the user's call, so this never triggers Auth.Login() itself.
		public bool BeginEpisode()
		{
			if (!Auth.IsAuthenticated())
			{
				Log.Warn("[Record] Observer recording skipped: not signed in to Lucky Hub. Sign in from the editor to record " +
					"robot/camera data; the JSON episode record is still written.");
				return false;
			}

			if (!Observer.IsRecording)
			{
				if (m_TaskIndex.Count == 0)
					foreach (string label in StackPlanner.AllStepLabels())
					{
						m_TaskIndex[label] = m_TaskIndex.Count;
						Observer.RegisterTask(m_TaskIndex[label], label);
					}
				Observer.StartRecording();   // opens the first episode
			}
			else
			{
				Observer.BeginEpisode();     // later episodes in the same recording
			}

			m_EpisodeOpen = true;
			m_Episodes++;
			Log.Info($"[Record] Observer episode {m_Episodes} started as {Auth.GetUsername()} ({m_TaskIndex.Count} pick labels): " +
				$"IsRecording={Observer.IsRecording}");
			return true;
		}

		// Label the frames that follow with this pick.
		public void SetStep(PlannedStep step)
		{
			if (!m_EpisodeOpen) return;
			if (step.IsBufferMove) return;   // buffer moves have no label; frames keep the label of the pick they unblock
			if (m_TaskIndex.TryGetValue(step.ToString(), out int index)) Observer.SetCurrentTaskIndex(index);
			else Log.Warn($"[Record] No Observer label registered for '{step}'");
		}

		// success = the evaluator's verdict, not the state machine's.
		public void EndEpisode(bool success, bool stopRecording)
		{
			if (!m_EpisodeOpen) return;
			Observer.EndCurrentEpisode(success);
			m_EpisodeOpen = false;
			if (stopRecording && Observer.IsRecording) Observer.StopRecording();
			Log.Info($"[Record] Observer episode {m_Episodes} ended (success={success}){(stopRecording ? ", recording stopped" : "")}");
		}
	}
}
