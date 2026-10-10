using System.Collections.Generic;
using Hazel;
using Hazel.Data;
using LuckySortStack.Planning;

namespace LuckySortStack.Episodes
{
	// Thin wrapper over Lucky's Observer (Hazel.Data.Observer), which records robot state, actions
	// and camera frames to its own dataset. This class only drives episode boundaries and labels:
	// one Observer task per distinct pick ("Pick Large Red -> RED[0]"), so the recording is
	// segmented per pick. Engine-dependent, so it lives in Episodes/ rather than Recording/
	//
	// API confirmed from Hazel-ScriptCore.dll; usage pattern from RobotSandbox (StartRecording, then
	// EndCurrentEpisode*(success)). Whether BeginEpisode() is needed after StartRecording() is
	// unverified (see LUCKY_API_NOTES §10), so Begin() logs IsRecording to make it visible.
	public sealed class ObserverSession
	{
		private readonly Dictionary<string, int> m_TaskIndex = new Dictionary<string, int>();
		private bool m_EpisodeOpen;

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

			if (!Observer.IsRecording) Observer.StartRecording();
			m_EpisodeOpen = true;
			Log.Info($"[Record] Observer recording as {Auth.GetUsername()}: IsRecording={Observer.IsRecording}");
			return true;
		}

		// Label the frames that follow with this pick. Task indices are shared across episodes by description.
		public void SetStep(PlannedStep step)
		{
			if (!m_EpisodeOpen) return;
			string description = step.ToString();
			if (!m_TaskIndex.TryGetValue(description, out int index))
			{
				index = m_TaskIndex.Count;
				m_TaskIndex[description] = index;
				Observer.RegisterTask(index, description);
			}
			Observer.SetCurrentTaskIndex(index);
		}

		// success = the evaluator's verdict, not the state machine's.
		public void EndEpisode(bool success, bool stopRecording)
		{
			if (!m_EpisodeOpen) return;
			Observer.EndCurrentEpisode(success);
			m_EpisodeOpen = false;
			if (stopRecording && Observer.IsRecording) Observer.StopRecording();
			Log.Info($"[Record] Observer episode ended (success={success}){(stopRecording ? ", recording stopped" : "")}");
		}
	}
}
