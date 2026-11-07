"""Pure artifact-selection regression checks; no ROS entities or navigation."""
import hashlib,json,pathlib,sys,tempfile,unittest
from unittest import mock
NAV=pathlib.Path(__file__).resolve().parent;sys.path.insert(0,str(NAV));import matrix_navigation as m
class BindingTests(unittest.TestCase):
 def setUp(self):
  self.t=tempfile.TemporaryDirectory();self.addCleanup(self.t.cleanup);self.p=pathlib.Path(self.t.name)/'install';self.p.mkdir();self.files={}
  for n in ['lib/nav2_lifecycle_manager/lifecycle_manager','lib/libnav2_lifecycle_manager_core.so']:
   q=self.p/n;q.parent.mkdir(parents=True,exist_ok=True);q.write_bytes(n.encode());self.files[n]=m.sha(q)
  self.r=self.p.parent/'receipt.json';self.data={'schema':'matrix.lifecycle.build_receipt.v1','install_prefix':str(self.p),'artifact_sha256':self.files,'source_manifest':{'source_commit':'d'*40}};self.save()
 def save(self):self.r.write_text(json.dumps(self.data))
 def test_default_and_pair(self):
  self.assertIsNone(m.lifecycle_artifacts(None,None))
  with self.assertRaises(ValueError):m.lifecycle_artifacts(self.p,None)
 def test_exact_pair_and_receipt(self):
  got=m.lifecycle_artifacts(self.p,self.r);self.assertEqual(got['artifact_sha256'],self.files);self.assertEqual(got['receipt_sha256'],m.sha(self.r))
 def test_mutation_is_rejected_on_revalidation(self):
  m.lifecycle_artifacts(self.p,self.r);(self.p/'lib/libnav2_lifecycle_manager_core.so').write_bytes(b'changed')
  with self.assertRaises(ValueError):m.lifecycle_artifacts(self.p,self.r)
 def test_external_or_symlink_artifact_is_rejected(self):
  outside=self.p.parent/'outside';outside.write_bytes(b'outside');q=self.p/'lib/libnav2_lifecycle_manager_core.so';q.unlink();q.symlink_to(outside);self.files['lib/libnav2_lifecycle_manager_core.so']=m.sha(q);self.save()
  with self.assertRaises(ValueError):m.lifecycle_artifacts(self.p,self.r)
 def test_wrong_prefix_and_missing_core_are_rejected(self):
  self.data['install_prefix']=str(self.p.parent);self.save()
  with self.assertRaises(ValueError):m.lifecycle_artifacts(self.p,self.r)
  self.data['install_prefix']=str(self.p);self.files.pop('lib/libnav2_lifecycle_manager_core.so');self.save()
  with self.assertRaises(ValueError):m.lifecycle_artifacts(self.p,self.r)
if __name__=='__main__':unittest.main(verbosity=2)
