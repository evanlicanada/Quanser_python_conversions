# Quanser Simulink to Python conversions

Current Owner: Evan

## Brief Description
My role was to convert the existing quanser simulink labs (found in Quanser Academic Resources under `6_teaching/3_robotics`) into python equivelents

## Organization
The repo is mainly organized to match the structure of the original repo, with `sp#_name/l#_lessonName`, then `digital_twin` and `hardware` folders for the virtual and real versions of the labs. You may notice it skips from sp0 to sp4, this is because only sp0, sp4, sp5 and sp6 use the QArm (see `6_teaching\3_Robotics\Getting Started.png`).

Three additional folders exist:
- `Lab_procedure_sources` contains the `.docx` source files for the lab procedure pdf's
- `modified_library` contains copies of classes from quanser's libraries. It retains the files names. Changes have been applied to those classes and they are used in the other code.
- `templates` contains a `.docx` and a `.py` templates to show how the lab procedures and lab code should be formatted

Other files:
- `play.py` is not particularly useful, but I retained it as a reference program
- `AGENTS.md` a useful file if you want to quickly give an AI agent the info it needs; make sure it reads this before performing changes to the repo.

## Modified Quanser Libraries
As mentioned, `modified_library` contains edited versions of classes from the Quanser libraries. As such before properly implementing these, make sure that the import paths are updated. The exact changes are outlined in the `CHANGES.md` file located in the folder, try to keep this updated.

## Submission to Quanser
For me, Quanser wanted the submission in a shared onedrive folder, so while this repo is my main storage, I would periodically upload new lab scripts and files to the shared folder.