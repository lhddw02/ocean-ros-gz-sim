#include "LandingReplay.hh"

#include <QQuickItem>
#include <gz/msgs/boolean.pb.h>
#include <gz/plugin/Register.hh>

LandingReplay::LandingReplay()
{
  // Transport callbacks run outside the GUI thread. All QML state changes are
  // queued back onto this object's Qt thread.
  QObject::connect(this, &LandingReplay::TransportStatus,
    this, &LandingReplay::ApplyStatus, Qt::QueuedConnection);
  this->responseTimer.setSingleShot(true);
  QObject::connect(&this->responseTimer, &QTimer::timeout, this, [this]()
  {
    this->busy = false;
    this->awaitingAcknowledgement = false;
    this->statusText = "No reset response. Press Replay landing to try again.";
    emit this->StatusChanged();
  });
}

LandingReplay::~LandingReplay()
{
  this->node.Unsubscribe(this->statusTopic);
}

void LandingReplay::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Landing demo";
  // An explicit lower-case object binding avoids the QML component type
  // LandingReplay shadowing the upper-case context property of the same name.
  if (auto *item = this->PluginItem())
    item->setProperty("control", QVariant::fromValue(static_cast<QObject *>(this)));
  if (_pluginElem)
  {
    if (const auto *elem = _pluginElem->FirstChildElement("topic"))
      if (elem->GetText())
        this->topic = elem->GetText();
    if (const auto *elem = _pluginElem->FirstChildElement("status_topic"))
      if (elem->GetText())
        this->statusTopic = elem->GetText();
  }
  this->publisher = this->node.Advertise<gz::msgs::Boolean>(this->topic);
  const bool subscribed = this->node.Subscribe(
    this->statusTopic, &LandingReplay::OnStatus, this);
  if (!this->publisher || !subscribed)
  {
    this->busy = false;
    this->statusText = "Unable to connect the replay controls.";
    emit this->StatusChanged();
  }
  else
  {
    this->responseTimer.start(30000);
  }
}

QString LandingReplay::StatusText() const
{
  return this->statusText;
}

bool LandingReplay::Busy() const
{
  return this->busy;
}

void LandingReplay::Replay()
{
  if (this->busy)
    return;
  if (!this->publisher.HasConnections())
  {
    this->statusText = "Landing controller is not connected. Try again shortly.";
    emit this->StatusChanged();
    return;
  }
  gz::msgs::Boolean request;
  request.set_data(true);
  if (!this->publisher.Publish(request))
  {
    this->statusText = "Could not send the reset request. Please try again.";
    emit this->StatusChanged();
    return;
  }
  this->busy = true;
  this->awaitingAcknowledgement = true;
  this->statusText = "Reset requested...";
  this->responseTimer.start(30000);
  emit this->StatusChanged();
}

void LandingReplay::OnStatus(const gz::msgs::StringMsg &_message)
{
  emit this->TransportStatus(QString::fromStdString(_message.data()));
}

void LandingReplay::ApplyStatus(const QString &_status)
{
  if (_status == "starting" || _status == "resetting" ||
      _status == "arming" || _status == "resuming")
  {
    this->awaitingAcknowledgement = false;
    this->busy = true;
    if (_status == "starting")
      this->statusText = "Waiting for the landing camera...";
    else if (_status == "resetting")
      this->statusText = "Resetting the scene...";
    else if (_status == "resuming")
      this->statusText = "Starting the new landing...";
    else
      this->statusText = "Preparing the drone...";
    this->responseTimer.start(30000);
  }
  else if (_status == "ready" || _status == "replaying" || _status == "landed")
  {
    if (this->awaitingAcknowledgement)
      return;
    this->responseTimer.stop();
    this->busy = false;
    if (_status == "landed")
      this->statusText = "Touchdown complete. Ready to replay.";
    else if (_status == "ready")
      this->statusText = "Ready to replay.";
    else
      this->statusText = "Landing in progress. Replay anytime.";
  }
  else if (_status.startsWith("error:"))
  {
    this->responseTimer.stop();
    this->awaitingAcknowledgement = false;
    this->busy = false;
    this->statusText = QString("Reset failed: %1").arg(
      _status.mid(6).trimmed().left(180));
  }
  else
  {
    return;
  }
  emit this->StatusChanged();
}

GZ_ADD_PLUGIN(LandingReplay, gz::gui::Plugin)
