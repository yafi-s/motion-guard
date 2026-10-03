import unittest
import numpy as np
import pandas as pd
from scenarios import independent_pair,slice_tracks


class EvaluatorTests(unittest.TestCase):
    def test_crossing_between_samples_is_missed_by_sampled_baseline(self):
        a=np.array([[-2.,0],[2.,0]])
        b=np.zeros((2,2))
        continuous,sampled,replay=independent_pair(a,b,.5)
        self.assertTrue(continuous)
        self.assertFalse(sampled)
        self.assertEqual(replay['nearest_fraction'],.5)

    def test_stationary_tangent_and_uncertainty_envelope(self):
        a=np.array([[0.,2],[0.,2]])
        b=np.zeros((2,2))
        self.assertFalse(independent_pair(a,b,1.)[0])
        self.assertTrue(independent_pair(a,b,2.)[0])
        self.assertTrue(independent_pair(a,b,3.)[0])

    def test_incomplete_and_duplicate_tracks_are_excluded(self):
        df=pd.DataFrame({'track_id':['a','a','a','b','b'], 'timestep':[0,1,2,0,2],
                         'position_x':[0]*5,'position_y':[0]*5,'object_type':['vehicle']*5})
        tracks,excluded=slice_tracks(df,0,2)
        self.assertEqual(len(tracks),1)
        self.assertEqual(excluded['missing_or_duplicate_frames'],1)
        duplicate=pd.concat([df,df.iloc[[0]]],ignore_index=True)
        self.assertEqual(slice_tracks(duplicate,0,2)[0],[])


if __name__=='__main__':
    unittest.main()
