/*
 * GuLinux Planetary Imager - https://github.com/GuLinux/PlanetaryImager
 * Copyright (C) 2017  Marco Gulino <marco@gulinux.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef EDITROIDIALOG_H
#define EDITROIDIALOG_H

#include <QDialog>
#include "c++/dptr.h"

class EditROIDialog : public QDialog
{
    Q_OBJECT
public:
    EditROIDialog(QWidget *parent = 0);
    ~EditROIDialog();
public slots:
  void setResolution(const QSize &resolution);
  void setCurrentROI(const QRect &roi);
signals:
  void roiSelected(const QRect &roi);
private:
  DPTR
};

#endif // EDITROIDIALOG_H
