#include <atomic>
#include <functional>
#include <QGuiApplication>
#include <QQmlComponent>
#include <QQmlContext>
#include <QQmlExpression>
#include <QQuickItem>
#include <QQuickView>
#include <QtTest/QTest>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>
#include "LandingReplay.hh"

class ReplayButtonTest : public QObject
{
  Q_OBJECT
  private slots: void clickPublishesOneRequest()
  {
    const std::string topic = "/landing/test/replay/" +
      std::to_string(QCoreApplication::applicationPid());
    const std::string statusTopic = topic + "/status";
    tinyxml2::XMLDocument document;
    const std::string config = "<plugin><topic>" + topic +
      "</topic><status_topic>" + statusTopic + "</status_topic></plugin>";
    QCOMPARE(document.Parse(config.c_str()), tinyxml2::XML_SUCCESS);

    std::atomic_int requests{0};
    gz::transport::Node observer;
    std::function<void(const gz::msgs::Boolean &)> receive =
      [&requests](const gz::msgs::Boolean &message)
      {
        if (message.data())
          ++requests;
      };
    QVERIFY(observer.Subscribe(topic, receive));
    auto statusPublisher = observer.Advertise<gz::msgs::StringMsg>(statusTopic);
    LandingReplay plugin;
    plugin.LoadConfig(document.RootElement());
    QQuickView view;
    // Match gz::gui::Plugin::Load: a child context owns the plugin binding.
    auto *context = new QQmlContext(view.engine()->rootContext(), &view);
    context->setContextProperty("LandingReplay", &plugin);
    QCOMPARE(plugin.property("statusText").toString(), plugin.StatusText());
    QQmlExpression binding(context, nullptr, "LandingReplay.statusText");
    QCOMPARE(binding.evaluate().toString(), plugin.StatusText());
    const QUrl qml("qrc:/LandingReplay/LandingReplay.qml");
    QQmlComponent component(view.engine(), qml);
    auto *item = qobject_cast<QQuickItem *>(component.create(context));
    QVERIFY(item);
    item->setProperty("control", QVariant::fromValue(static_cast<QObject *>(&plugin)));
    view.setContent(qml, &component, item);
    QCOMPARE(view.status(), QQuickView::Ready);
    view.resize(350, 150);
    view.show();
    auto *button = view.rootObject()->findChild<QQuickItem *>("replayButton");
    QVERIFY(button);
    QVERIFY(!button->isEnabled());
    QTRY_VERIFY_WITH_TIMEOUT(statusPublisher.HasConnections(), 5000);
    gz::msgs::StringMsg status;
    status.set_data("ready");
    QVERIFY(statusPublisher.Publish(status));
    QTRY_VERIFY_WITH_TIMEOUT(button->isEnabled(), 5000);
    // Publisher discovery is separate from the status subscription.
    QTest::qWait(500);
    const QPoint buttonCenter = button->mapToScene(
      QPointF(button->width() / 2, button->height() / 2)).toPoint();
    QTest::mouseClick(&view, Qt::LeftButton, Qt::NoModifier, buttonCenter);
    QTRY_COMPARE_WITH_TIMEOUT(requests.load(), 1, 5000);
    QVERIFY(!button->isEnabled());
    QTest::mouseClick(&view, Qt::LeftButton, Qt::NoModifier, buttonCenter);
    QTest::qWait(200);
    QCOMPARE(requests.load(), 1);
    // Old periodic ready messages must not reopen the button before an ack.
    QVERIFY(statusPublisher.Publish(status));
    QTest::qWait(200);
    QVERIFY(!button->isEnabled());
    status.set_data("resetting");
    QVERIFY(statusPublisher.Publish(status));
    QTRY_COMPARE_WITH_TIMEOUT(plugin.StatusText(), QString("Resetting the scene..."), 5000);
    status.set_data("arming");
    QVERIFY(statusPublisher.Publish(status));
    QTRY_COMPARE_WITH_TIMEOUT(plugin.StatusText(), QString("Preparing the drone..."), 5000);
    QVERIFY(!button->isEnabled());
    status.set_data("replaying");
    QVERIFY(statusPublisher.Publish(status));
    QTRY_VERIFY_WITH_TIMEOUT(button->isEnabled(), 5000);
    QTest::mouseClick(&view, Qt::LeftButton, Qt::NoModifier, buttonCenter);
    QTRY_COMPARE_WITH_TIMEOUT(requests.load(), 2, 5000);
  }
};

int main(int argc, char **argv)
{
  QGuiApplication application(argc, argv);
  ReplayButtonTest test;
  return QTest::qExec(&test, argc, argv);
}

#include "replay_button_test.moc"
