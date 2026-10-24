#ifndef OCEAN_LANDING_GUI_LANDING_REPLAY_HH_
#define OCEAN_LANDING_GUI_LANDING_REPLAY_HH_

#include <string>
#include <QTimer>
#include <gz/gui/qt.h>
#include <gz/gui/Plugin.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

/// A one-shot replay request. The ROS coordinator owns the reset sequence.
class LandingReplay final : public gz::gui::Plugin
{
  Q_OBJECT
  Q_PROPERTY(QString statusText READ StatusText NOTIFY StatusChanged)
  Q_PROPERTY(bool busy READ Busy NOTIFY StatusChanged)

  public: LandingReplay();
  public: ~LandingReplay() override;
  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;
  public: QString StatusText() const;
  public: bool Busy() const;

  public slots: void Replay();

  signals: void StatusChanged();
  signals: void TransportStatus(const QString &_status);

  private: void OnStatus(const gz::msgs::StringMsg &_message);
  private: void ApplyStatus(const QString &_status);
  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher publisher;
  private: std::string topic{ "/landing/replay" };
  private: std::string statusTopic{ "/landing/replay_status" };
  private: QString statusText{ "Connecting to landing controller..." };
  private: bool busy{true};
  // Ignore stale ready/replaying messages until the reset is acknowledged.
  private: bool awaitingAcknowledgement{false};
  private: QTimer responseTimer;
};

#endif
