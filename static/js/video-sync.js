// Keeps each side-by-side video pair playing together, and pauses pairs that are
// scrolled out of view so several looping clips don't burn CPU in the background.
//
// Play/pause are mirrored, but seek position deliberately is not: the two clips in
// a pair have different durations, so a shared timeline would imply a
// frame-to-frame correspondence that does not exist. Looping is left to the
// native `loop` attribute, so the sides drift apart over a long visit -- harmless
// for the same reason.

document.addEventListener('DOMContentLoaded', function () {
  var groups = Array.prototype.slice.call(document.querySelectorAll('[data-sync-group]'));

  groups.forEach(function (group) {
    var videos = Array.prototype.slice.call(group.querySelectorAll('video'));
    if (videos.length < 2) return;

    // mirroring: guards against a mirrored play()/pause() echoing back forever.
    // suppress: set while we drive the pair ourselves, so scroll-driven pauses
    //   aren't mistaken for the viewer pausing the pair.
    // userPaused: remembers a deliberate pause, so scrolling back into view
    //   doesn't override it.
    var mirroring = false;
    var suppress = false;
    var userPaused = false;

    function play(video) {
      var playing = video.play();
      if (playing && playing.catch) playing.catch(function () {});
    }

    function mirror(source, action) {
      if (mirroring) return;
      mirroring = true;
      videos.forEach(function (other) {
        if (other === source) return;
        if (action === 'play') {
          play(other);
        } else {
          other.pause();
        }
      });
      mirroring = false;
    }

    videos.forEach(function (video) {
      video.addEventListener('play', function () {
        if (!suppress) userPaused = false;
        mirror(video, 'play');
      });
      video.addEventListener('pause', function () {
        if (!suppress) userPaused = true;
        mirror(video, 'pause');
      });
    });

    if (!('IntersectionObserver' in window)) return;

    var observer = new IntersectionObserver(function (entries) {
      entries.forEach(function (entry) {
        suppress = true;
        videos.forEach(function (video) {
          if (entry.isIntersecting && !userPaused) {
            play(video);
          } else if (!entry.isIntersecting) {
            video.pause();
          }
        });
        // play()/pause() queue their events, so release the guard after they fire.
        setTimeout(function () { suppress = false; }, 0);
      });
    }, { threshold: 0.2 });

    observer.observe(group);
  });
});
