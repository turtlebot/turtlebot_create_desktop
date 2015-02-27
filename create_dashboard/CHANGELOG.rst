^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package create_dashboard
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.1 (2015-02-27)
------------------
* create_dashboard: Fix some python syntax errors
  Even if this file isn't used, it will be bytecompiled when packaged for Fedora. This will fail if there are syntax errors in the file, so even if it is not used or has functional errors, it needs to be syntactically correct if it is in the repository.
* Contributors: Scott K Logan

2.3.0 (2014-12-01)
------------------

2.2.0 (2013-08-30)
------------------
* adds bugtracker and repo info to package.xml
* fixes grey battery status issue

2.1.0 (2013-07-18)
------------------
* Fully catkinized
* Hydro beta release
