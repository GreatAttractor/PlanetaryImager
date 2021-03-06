/*
 * Copyright (C) 2016  Marco Gulino <marco@gulinux.net>
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

#ifndef FRAMESFORWARDER_H
#define FRAMESFORWARDER_H
#include "image_handlers/imagehandler.h"
#include "network/networkdispatcher.h"
#include "c++/dptr.h"
#include <QObject>
class FramesForwarder : public QObject, public ImageHandler
{
Q_OBJECT
public:
  typedef std::shared_ptr<FramesForwarder> ptr;
  FramesForwarder(const NetworkDispatcher::ptr &dispatcher);
  ~FramesForwarder();
  void handle(const Frame::ptr & frame) override;
  bool enabled() const;
private:
  DPTR
public slots:
  void setEnabled(bool enabled);
  void recordingMode(bool recording);
};

#endif // FRAMESFORWARDER_H
