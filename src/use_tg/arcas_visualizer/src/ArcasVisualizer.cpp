/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2014  sinosuke <email>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/


#include "ArcasVisualizer.h"
#include <QMessageBox>
#include <QFileDialog>
#include <QInputDialog>
#include <boost/lexical_cast.hpp>
#include <QStatusBar>

#define BUFFER_LENGTH 100000

using namespace std;
using namespace functions;
using boost::lexical_cast;
using boost::bad_lexical_cast;

ArcasVisualizer::ArcasVisualizer(int argc, char **argv, QWidget* parent, Qt::WindowFlags flags): 
QMainWindow(parent, flags), argc(argc), argv(argv), init_log_time(), curves(), curves_z(), distance_log(), distance_log_z(), position_log(),
t_log(), xy_dist(1.2), z_dist(0.3), node(NULL), uavs(), pos_log(), time_step(0.1), timer(NULL), count(0), logging(false)
{
  setupUi(this);
  
  // Make Qt connections
  connect(pushButton, SIGNAL(pressed()), SLOT(startStopLog()));
  connect(connect_ros, SIGNAL(triggered(bool)), SLOT(startStopLog()));
  connect(actionStart_Logging, SIGNAL(triggered(bool)), SLOT(startStopLog()));
  connect(actionSave_Log, SIGNAL(triggered(bool)), SLOT(saveLog()));
  connect(actionQuit, SIGNAL(triggered(bool)), SLOT(close()));
  connect(pushButton_2, SIGNAL(pressed()), SLOT(takeoff()));
  connect(pushButton_4, SIGNAL(pressed()), SLOT(land()));
  connect(pushButton_6, SIGNAL(pressed()), SLOT(takeoffDialog()));
  connect(pushButton_5, SIGNAL(pressed()), SLOT(emergencyStop()));
  
  enableButtons(false); 
//   connect(actionLoad, SIGNAL(triggered()), SLOT(open()));
  node = NULL;
  
  
  // Initialize variables
  logging = false;
  getConfigurationData(false);
}

ArcasVisualizer::ArcasVisualizer(const QMainWindow& ): QMainWindow()
{

}


ArcasVisualizer::~ArcasVisualizer()
{
  stopLogging();
  
  // Erase plots
  qwtPlot->detachItems();
  qwtPlot_2->detachItems();
  curves.clear();
  curves_z.clear();
  clearLogs();
  delete timer;
}

bool ArcasVisualizer::saveLog()
{
  bool ret = true;
  
  QString distance_file = QFileDialog::getSaveFileName(this, "Enter distance filename");
  QString position_file = QFileDialog::getSaveFileName(this, "Enter position filename");
  ret = saveDistance(distance_file.toStdString());
  if (ret) {
    ret = savePosition(position_file.toStdString());
  }
  return ret;
}

bool ArcasVisualizer::startLogging()
{
  bool ret_val = true;
  // First configure the ROS connections
  if (node == NULL) {
    // First create the node
    node = new Comms(argc, argv);
    logging = false;
  }
  if (!logging) {
    getConfigurationData(false);
    
    if (uavs.size() != 0) {
      logging = true;
      init_log_time.getTime();
      ros::NodeHandle n;
      node->startComms(uavs); // Start comms
      
      // No data variables for initializing purposes
      arcas_msgs::QuadStateEstimationWithCovariance pos;
      vector<Point3D> v_p3_void;
    
      clearLogs();
      
      // Plot initialization
      // Clear logs
      qwtPlot->detachItems();
      qwtPlot->autoReplot();
      qwtPlot->axisAutoScale(0);
      qwtPlot->axisAutoScale(1);
      curves.clear();
      distance_log.clear();
      t_log.clear();
      pos_log.clear();
      distance_log_z.clear();
      t_log.reserve(BUFFER_LENGTH);
      vector<double> no_data;
      vector<Point3D> no_point;
      
      
      
      cout << "N_uavs " << uavs.size() << endl;
      unsigned int c = 0;
      for (size_t i = 0; i < uavs.size(); i++) {
	pos_log.push_back(no_point);
	for (size_t j = i + 1; j < uavs.size(); j++, c++) {
	  // Configure the distances curves
	  ostringstream os;
	  os << "Distance_" << uavs.at(i) << "_" << uavs.at(j);
	  QwtPlotCurve *curve = new QwtPlotCurve(os.str().c_str());
	  
	  QPen pen(selectColor(c));
	  curve->setPen(pen);
	  curve->attach(qwtPlot);
	  curves.push_back(curve);
	  curve = new QwtPlotCurve(os.str().c_str());
	  curve->setPen(pen);
	  curve->attach(qwtPlot_2);
	  curves_z.push_back(curve);
	  
	  distance_log.push_back(no_data);
	  distance_log_z.push_back(no_data);
	  distance_log.at(distance_log.size() - 1).reserve(BUFFER_LENGTH);
	  distance_log_z.at(distance_log.size() - 1).reserve(BUFFER_LENGTH);
	  
	  qwtPlot->setAxisAutoScale(0);
	  qwtPlot->setAxisAutoScale(1);
	  qwtPlot_2->setAxisAutoScale(0);
	  qwtPlot_2->setAxisAutoScale(1);
	}
      }
      timer = new QTimer(this);
      connect(timer, SIGNAL(timeout()), this, SLOT(updateValues()));
  //     cout << "timer->start("<<time_step*1000<< ")" << endl;
      timer->start((int)(time_step * 1000));
      count = 0;
      enableButtons(true);
    } else {
      ret_val = false;
      QMessageBox::information(this, "Connect ROS", "Please fill the ID of the UAVs you want to connect (ex: 4,5)(ex 4-7,10).", QMessageBox::Ok);
    }
  }
  
  return ret_val;
}

void ArcasVisualizer::stopLogging()
{
  enableButtons(false);
  if (logging) {
    logging = false;
    node->shutdownComms();
  }
  delete timer;
  timer = NULL;
}

void ArcasVisualizer::getConfigurationData(bool message)
{
  if (!logging) {
    if (node != NULL) {
      uavs = getUAVs(lineEdit_6->text());
      node->setTopics(lineEdit_5->text().toStdString(), 
		      lineEdit->text().toStdString(),
		      lineEdit_2->text().toStdString(),
		      lineEdit_3->text().toStdString(),
		      lineEdit_4->text().toStdString(), lineEdit_13->text().toStdString(), lineEdit_14->text().toStdString());
      time_step = doubleSpinBox->value();
      xy_dist = doubleSpinBox_2->value();
      z_dist = doubleSpinBox_3->value();
      cout << "Time step: " << time_step << "\txy_dist = " << xy_dist << "\tz_dist" << z_dist << endl;
      
      if (message) {
	QMessageBox::information(this, "Confirm Parameters", "Parameters have been retrieved successfully.", QMessageBox::Ok);
      }
    } if (message) {
      QMessageBox::information(this, "Confirm Parameters", "Node not ready.", QMessageBox::Ok);
    }
  } else {
    if (message) {
      QMessageBox::information(this, "Confirm Parameters", "Configuration parameters cannot be changed while performing a log.", QMessageBox::Ok);
    }
  }
}

void ArcasVisualizer::updateValues()
{
  
  FormattedTime t;
  t.getTime();
  
//   ostringstream os;
//   os << "Getting Values " << count;
//   statusBar()->showMessage(QString(os.str().c_str()));
   
  size_t n_uavs = uavs.size();
  t_log.push_back(t - init_log_time);
  vector<Point3D> cp = node->getCurrentPos();
  vector<Point3D> cv = node->getCurrentVel();
  vector<Point3D> desired_vel = node->getDesiredVel();
  FormattedTime t2;
  t2.getTime();
//   cout << "Get data time: " << t2 - t << " " << " N_uavs = " << n_uavs << "UAVs : " << printVector(uavs) << endl;
  int c = 0;
  position_log.push_back(cp);
  for (size_t i = 0;  i < n_uavs ; i++) {
    for (size_t j = i + 1; j < n_uavs;j++, c++) {
      distance_log.at(c).push_back(cp.at(i).distance2d(cp.at(j)));
      distance_log_z.at(c).push_back(fabs(cp.at(i).z - cp.at(j).z));
      if (distance_log.at(c).at(distance_log.at(c).size() - 1) < xy_dist &&
	distance_log_z.at(c).at(distance_log_z.at(c).size() - 1) < z_dist) {
	cerr << "Emergency situation detected. Sending emergency stop signal.\n";
	emergencyStop();
      }
    }
  }
  
  // Update plots
  c = 0;
  for (unsigned int i = 0; i < n_uavs; i++) {
    for (unsigned int j = i + 1; j < n_uavs; j++, c++) {
      if (distance_log.at(c).size() > 0) {
	curves.at(c)->setRawSamples(&t_log.at(0), &(distance_log.at(c).at(0)), (int)t_log.size());
	curves_z.at(c)->setRawSamples(&t_log.at(0), &(distance_log_z.at(c).at(0)), (int)t_log.size());
      }
    }
  }
  qwtPlot->replot();
  qwtPlot_2->replot();

  count++;
  t2.getTime();
//   cout << "Get values time: " << t2 - t << endl;
}

void ArcasVisualizer::clearLogs()
{
  t_log.clear();
  distance_log.clear();
  pos_log.clear();
  position_log.clear();
  distance_log_z.clear();
}


vector< uint > ArcasVisualizer::getUAVs(QString s)
{
  vector<uint> ret_val;
  
  QRegExp rx("[\\s,]");
  QStringList list = s.split(rx, QString::SkipEmptyParts); 
  
  foreach(QString s_, list) {
    QStringList l = s_.split("-");
    if (l.size() == 2) {
      int low = -1;
      int high = -1;
      try
        {
	  low = lexical_cast<int>(l.at(0).toStdString());
	  high = lexical_cast<int>(l.at(1).toStdString());
	  
	  for (int i = low; i <= high && low > 0 && high > 0; i++) {
	    ret_val.push_back((uint)i);
	  }
        }
        catch(const bad_lexical_cast &)
        {
	  cerr << "Bad conversion!" << endl;
        }
    } else {
      try
        {
	  ret_val.push_back(lexical_cast<int>(s_.toStdString()));
	}
        catch(const bad_lexical_cast &)
        {
	  cerr << "Bad conversion!" << endl;
        }
    }
  }
  
  cout << "Uavs: " << functions::printVector(uavs) << endl;
  
  return ret_val;
}

void ArcasVisualizer::startStopLog()
{
  if (logging) {
    stopLogging();
    this->statusBar()->showMessage("Logging stopped");
  } else {
    if (startLogging()) {
      this->statusBar()->showMessage("Logging started");
    } else {
      this->statusBar()->showMessage("Could not start to log");
    }
  }
}


bool ArcasVisualizer::savePosition(const std::string& desired_file) const{
  bool ret_val = true;
  
  for (unsigned int i = 0; i < uavs.size() && ret_val; i++) {
    ostringstream oss;
    oss << desired_file << i;
    ofstream ofs;
    ofs.open(oss.str().c_str());
    ret_val = ofs.is_open();
    for (unsigned int j = 0; ret_val && j < position_log.size(); j++) {
      ofs << t_log.at(i) << " ";
      ofs << position_log.at(j).at(i).toString(false) << endl;
    }
    ofs.close();
  }
  return ret_val;
}

bool ArcasVisualizer::saveDistance(const string& distance_file) const
{
  ofstream ofs;
  ofs.open(distance_file.c_str());
  for (unsigned int i = 0;i < t_log.size(); i++) {
    ofs << t_log.at(i) << " ";
    for (unsigned int j = 0; j < distance_log.size(); j++) {
      ofs << distance_log.at(j).at(i) << " ";
    }
    ofs << endl;
  }
  ofs.close();
  
  ofstream ofs_z;
  string file_(distance_file);
  file_.append("_z");
  ofs_z.open(file_.c_str());
  for (unsigned int i = 0;i < t_log.size(); i++) {
    ofs_z << t_log.at(i) << " ";
    for (unsigned int j = 0; j < distance_log_z.size(); j++) {
      ofs_z << distance_log_z.at(j).at(i) << " ";
    }
    ofs_z << endl;
  }
  ofs_z.close();
  
  return true;
}

QColor ArcasVisualizer::selectColor(unsigned int i)
{
  QColor ret = Qt::blue;
  switch (i%8) {
    case 0:
    ret = Qt::blue;
    break;
    case 1:
    ret = Qt::red;
    break;
    case 2:
    ret = Qt::green;
    break;
    case 3:
    ret = Qt::cyan;
    break;
    case 4:
    ret = Qt::magenta;
    break;
    case 5:
    ret = Qt::yellow;
    break;
    case 6:
    ret = Qt::black;
    break;
    case 7:
    default:
    ret = Qt::gray;
    break;
  }
  return ret;
}

void ArcasVisualizer::emergencyStop()
{
  statusBar()->showMessage("Emergency stop sent.\n");
  for (unsigned int i = 0; i < uavs.size(); i++) {
    node->emergencyStop(i);
  }
}

void ArcasVisualizer::land()
{
  vector<boost::thread*> v;
  this->statusBar()->showMessage("Landing all UAVs");
  repaint();
  for (unsigned int i = 0; i < uavs.size(); i++) {
    boost::function0<void> f = boost::bind<void>(&Comms::land, node, i);
    boost::thread *new_thread = new boost::thread(f);
    v.push_back(new_thread);
  }
  
  for (unsigned int i = 0; i < v.size(); i++) {
    v.at(i)->join();
    delete v.at(i);
  }
  v.clear();
  
  this->statusBar()->showMessage("All UAVs landed ok");
}

void ArcasVisualizer::takeoffDialog()
{
  QString uavs_takeoff = QInputDialog::getText(this, "Takeoff dialog", "Enter the UAVs to takeoff");
  vector<uint> uavs_t = getUAVs(uavs_takeoff);
  takeoff(uavs_t);
}

void ArcasVisualizer::takeoff() {
  takeoff(uavs);
}

void ArcasVisualizer::takeoff(const std::vector<uint> uavs_)
{
  bool cancel = false;
  // We will compare the ids of the UAVs to takeoff with the ones already registered
  for (unsigned int i = 0; i < uavs_.size() && !cancel; i++) {
    for (unsigned int j = 0; j < uavs.size() && !cancel; j++) {
      if (uavs_.at(i) == uavs.at(j)) {
	ostringstream os;
	os << "Press OK to takeoff the UAV " << uavs.at(j)<< "\n Cancel will Abort all pending takeoffs.\n";
	if (QMessageBox::information(this, "Takeoff", os.str().c_str(), QMessageBox::Ok, QMessageBox::Cancel) != QMessageBox::Ok) {
	  cancel = true;
	} else {
	  node->takeoff(j);
	}
	repaint();
      }
    }
  }
  if (!cancel) {
    this->statusBar()->showMessage("All UAVs are flying ok");
  } else {
    this->statusBar()->showMessage("Takeoff canceled!");
  }
}

void ArcasVisualizer::enableButtons(bool en)
{
  pushButton_2->setEnabled(en); // Takeoff
  pushButton_4->setEnabled(en); // Land
  pushButton_5->setEnabled(en); // Emergency stop
  pushButton_6->setEnabled(en); // Takeoff Dialog
  pushButton_9->setEnabled(en); // Emergency stop
}

