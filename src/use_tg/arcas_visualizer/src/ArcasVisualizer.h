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


#ifndef ARCASVISUALIZER_H
#define ARCASVISUALIZER_H

#include <functions/functions.h>
#include <functions/Point3D.h>
#include <functions/FormattedTime.h>

#include "Comms.h"

// Qt includes
#include <QTimer>
#include <qwt/qwt_plot_curve.h>
#include <qwt/qwt_plot_marker.h>
#include <qwt/qwt_symbol.h>
#include "ui_arcas_coordination_center.h"

class ArcasVisualizer:public QMainWindow, Ui::MainWindow
{
  Q_OBJECT
public:
    ArcasVisualizer(int argc, char **argv, QWidget* parent = 0, Qt::WindowFlags flags = 0);
    virtual ~ArcasVisualizer();
    ArcasVisualizer(const QMainWindow& );
    
private slots:
  bool saveLog();
  void startStopLog();
  
  //! @brief Confirms the configuration data in the Configuration tab
  void getConfigurationData(bool message = true);
  
  void updateValues();
  
  void takeoff();
  
  void land();
  
  void takeoffDialog();
  
  void emergencyStop();
  
private:
  //! @brief Clears the data contained in the logs
  void clearLogs();
  //! @brief Saves the velocities info into separate files (both the desired and the orca)
//   bool saveVelocities(const std::string& desired_file, const std::string& orca_file) const;
  bool savePosition(const std::string& position_file) const;
  //! @brief Saves the distance of each pair into a file
  bool saveDistance(const std::string& distance_file) const;
  //! @brief Translates a string to the number of UAVs
  std::vector<uint> getUAVs(QString s);
  //! @brief Returns a color depending on the input number
  static QColor selectColor(unsigned int i);
  //! @brief Takes off sequentially the uavs in the vector
  void takeoff(const std::vector< uint > uavs);
  
  void enableButtons(bool en);
  
  void stopLogging();
  bool startLogging();
  
  int argc;
  char **argv;
  functions::FormattedTime init_log_time;
  
  // Curves
  std::vector<QwtPlotCurve *> curves;
  std::vector<QwtPlotCurve *> curves_z;
  std::vector<std::vector <double> > distance_log;
  std::vector<std::vector <double> > distance_log_z;
  std::vector<std::vector<functions::Point3D> > position_log;
  std::vector<double> t_log;
  
  // Automatic emergency stop!!
  double xy_dist, z_dist;
  
  // Communication stuff
  Comms *node;
  std::vector<uint> uavs;
//   std::vector<std::vector<functions::Point3D> > desired_vel_log, orca_vel_log;
  std::vector<std::vector<functions::Point3D> > pos_log;
  
  // Configuration
  double time_step;
  QTimer *timer;
  unsigned int count;
  
  // Flags
  bool logging;
};

#endif // ARCASVISUALIZER_H
